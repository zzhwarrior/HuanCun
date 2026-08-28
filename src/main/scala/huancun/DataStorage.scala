/** *************************************************************************************
  * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
  * Copyright (c) 2020-2021 Peng Cheng Laboratory
  *
  * XiangShan is licensed under Mulan PSL v2.
  * You can use this software according to the terms and conditions of the Mulan PSL v2.
  * You may obtain a copy of Mulan PSL v2 at:
  * http://license.coscl.org.cn/MulanPSL2
  *
  * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
  * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
  * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
  *
  * See the Mulan PSL v2 for more details.
  * *************************************************************************************
  */

// See LICENSE.SiFive for license details.

package huancun

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import huancun.utils.SRAMWrapper
import utility._

// Request bundle for TCM SRAM access. Under the unified-bank layout, TCM
// addresses translate to a (way, set) pair before entering DataStorage —
// TcmSinkA owns that translation. wdata is a single stack's width (= one
// beat) since one TCM beat touches exactly one stack. wmask is a per-bank
// write-enable within the target stack: bit i set ⇒ bank i's 8 bytes will
// be updated, bit i clear ⇒ bank i's contents are preserved (RMW behaviour
// for aligned sub-beat writes). All-ones for full 64B writes (e.g. DMA).
class TCMReq(implicit p: Parameters) extends HuanCunBundle {
  val wen   = Bool()
  val way   = UInt(wayBits.W)
  val set   = UInt(setBits.W)
  val wdata = UInt((beatBytes * 8).W)
  val wmask = UInt((beatBytes / 8).W)   // one bit per 8-byte bank; = stackSize
}

class DataStorage(implicit p: Parameters) extends HuanCunModule {
  val io = IO(new Bundle() {
    val sourceC_raddr = Flipped(DecoupledIO(new DSAddress))
    val sourceC_rdata = Output(new DSData)
    val sinkD_waddr = Flipped(DecoupledIO(new DSAddress))
    val sinkD_wdata = Input(new DSData)
    val sourceD_raddr = Flipped(DecoupledIO(new DSAddress))
    val sourceD_rdata = Output(new DSData)
    val sourceD_waddr = Flipped(DecoupledIO(new DSAddress))
    val sourceD_wdata = Input(new DSData)
    val sinkC_waddr = Flipped(DecoupledIO(new DSAddress))
    val sinkC_wdata = Input(new DSData)
    val ecc = Valid(new EccInfo)
    // TCM port (only active when tcmEnabled). Under the unified-bank layout
    // TCM requests join the same 6-way arbiter as cache requests; the read
    // response comes back through a dedicated selector so it doesn't share
    // sourceD/sourceC read-data paths.
    val tcm_req   = if (tcmEnabled) Some(Flipped(DecoupledIO(new TCMReq))) else None
    val tcm_rdata = if (tcmEnabled) Some(Output(UInt((beatBytes * 8).W)))   else None
  })

  /* Define some internal parameters */
  // nrCacheStacks is 1 when TCM is enabled (8 cache banks) and 2 otherwise (16 cache banks).
  // Reducing nrCacheStacks by half keeps the physical SRAM depth constant while freeing
  // 8 banks for TCM.
  val nrStacks = nrCacheStacks
  // stackBits is clamped to ≥1 to avoid 0-width wire slices; the req() function
  // handles the nrStacks==1 case explicitly and never uses this value as a slice width.
  val stackBits = math.max(log2Ceil(nrStacks), 1)
  val bankBytes = 8
  val rowBytes = nrStacks * beatBytes
  val nrRows = sizeBytes / rowBytes
  val nrBanks = rowBytes / bankBytes
  val rowBits = log2Ceil(nrRows)
  val stackSize = nrBanks / nrStacks
  val sramSinglePort = true

  // Suppose * as one bank
  // All banks can be grouped by nrStacks. We call such group as stack
  //     one row ==> ******** ******** ******** ********
  // If there's no conflict, one row can be accessed in parallel by nrStacks

  def dataCode: Code = Code.fromString(p(HCCacheParamsKey).dataECC)

  val eccBits = dataCode.width(8 * bankBytes) - 8 * bankBytes
  println(s"Data ECC bits:$eccBits")

  val bankedData = Seq.fill(nrBanks) {
    Module(
      new SRAMWrapper(
        gen = UInt((8 * bankBytes).W),
        set = nrRows,
        n = cacheParams.sramDepthDiv,
        clk_div_by_2 = cacheParams.sramClkDivBy2
      )
    )
  }
  val dataEccArray = if (eccBits > 0) {
    Seq.fill(nrStacks) {
      Module(new SRAMWrapper(
        gen = UInt((eccBits * stackSize).W),
        set = nrRows,
        n = cacheParams.sramDepthDiv,
        clk_div_by_2 = cacheParams.sramClkDivBy2
      ))
    }
  } else null

  // TCM shares 'bankedData' with the cache side; it is admitted into the
  // per-stack arbiter as an additional highest-priority request source (see
  // reqs sequence below). No separate SRAM instances exist any more.

  val stackRdy = if (cacheParams.sramClkDivBy2) {
    RegInit(VecInit(Seq.fill(nrStacks) {
      true.B
    }))
  } else VecInit(Seq.fill(nrStacks) {
    true.B
  })

  /* Convert to internal request signals */
  class DSRequest extends HuanCunBundle {
    val wen = Bool()
    val index = UInt((rowBytes * 8).W)
    val bankSel = UInt(nrBanks.W)
    val bankSum = UInt(nrBanks.W)
    val bankEn = UInt(nrBanks.W)
    val data = Vec(nrBanks, UInt((8 * bankBytes).W))
    // Per-bank write-enable mask, ANDed with the derived per-bank wen. Cache
    // paths always set this to all-1s (full-line writes). TCM partial-beat
    // writes narrow it down so only the byte-lanes covered by the TL a.mask
    // actually update SRAM.
    val bankWMask = UInt(nrBanks.W)
  }

  def req(wen: Bool, addr: DecoupledIO[DSAddress], data: DSData): DSRequest = {
    // Remap address
    // [beat, set, way, block] => [way, set, beat, block]
    //                            [index, stack, block]
    val innerAddr = Cat(addr.bits.way, addr.bits.set, addr.bits.beat)
    // When nrStacks==1 there is no stack dimension: every request goes to stack 0.
    val innerIndex = if (nrStacks == 1) innerAddr else innerAddr >> stackBits
    val stackIdx   = if (nrStacks == 1) 0.U(1.W)  else innerAddr(stackBits - 1, 0)
    val stackSel = UIntToOH(stackIdx, stackSize) // Select which stack to access

    val out = Wire(new DSRequest)
    val accessVec = Cat(
      Seq
        .tabulate(nrStacks) { i =>
          !out.bankSum((i + 1) * stackSize - 1, i * stackSize).orR
        }
        .reverse
    )
    addr.ready := accessVec(stackIdx) && stackRdy(stackIdx)

    out.wen := wen
    out.index := innerIndex
    // FillInterleaved: 0010 => 00000000 00000000 11111111 00000000
    out.bankSel := Mux(addr.valid, FillInterleaved(stackSize, stackSel), 0.U) // TODO: consider mask
    out.bankEn := Mux(addr.bits.noop || !stackRdy(stackIdx),
      0.U,
      out.bankSel & FillInterleaved(stackSize, accessVec)
    )
    out.data := Cat(Seq.fill(nrStacks)(data.data)).asTypeOf(out.data.cloneType)
    // Cache paths never do partial writes: full cacheline replace only.
    out.bankWMask := ~0.U(nrBanks.W)
    out
  }

  /* Arbitrates r&w by bank according to priority */
  val sourceC_req = req(false.B, io.sourceC_raddr, io.sourceC_rdata)
  val sourceD_rreq = req(false.B, io.sourceD_raddr, io.sourceD_rdata)
  val sourceD_wreq = req(true.B, io.sourceD_waddr, io.sourceD_wdata)
  val sinkD_wreq   = req(true.B, io.sinkD_waddr, io.sinkD_wdata)
  val sinkC_req    = req(true.B, io.sinkC_waddr, io.sinkC_wdata)

  // TCM: build a DSAddress-shaped wire from the TCMReq bundle and reuse req().
  // TCM is placed at the *highest* priority in the reqs sequence so scratchpad
  // access has deterministic latency (a Step-3 anti-starvation counter on the
  // cache sources will keep cache from being locked out on the same stack).
  val tcm_req_opt: Option[DSRequest] = if (tcmEnabled) {
    val tr    = io.tcm_req.get
    val addrW = Wire(DecoupledIO(new DSAddress))
    val dataW = Wire(new DSData)
    addrW.valid       := tr.valid
    addrW.bits.way    := tr.bits.way
    addrW.bits.set    := tr.bits.set
    addrW.bits.beat   := 0.U
    addrW.bits.write  := tr.bits.wen
    addrW.bits.noop   := false.B
    tr.ready          := addrW.ready
    dataW.data        := tr.bits.wdata
    dataW.corrupt     := false.B
    val ds = req(tr.bits.wen, addrW, dataW)
    // Narrow bankWMask to the byte-lane mask. TL a.bits.mask is per-byte;
    // TcmSinkA has already OR-reduced it to per-8B-bank (= stackSize bits).
    // A TCM request only targets one stack (chosen via set[0]) so we place
    // wmask at the correct stack's bank positions and leave the other stack
    // as zero. Cache stack: banks [0..stackSize); other: banks [stackSize..).
    if (nrStacks == 2) {
      val stackIdx = tr.bits.set(0)
      val wmaskLo  = Mux(stackIdx, 0.U(stackSize.W), tr.bits.wmask)
      val wmaskHi  = Mux(stackIdx, tr.bits.wmask,   0.U(stackSize.W))
      ds.bankWMask := Cat(wmaskHi, wmaskLo)
    } else {
      // nrStacks==1: no stack dimension, all banks are one stack.
      ds.bankWMask := tr.bits.wmask
    }
    Some(ds)
  } else None

  val cacheReqs = Seq(sourceC_req, sinkC_req, sinkD_wreq, sourceD_wreq, sourceD_rreq)
  val reqs = tcm_req_opt.toSeq ++ cacheReqs
    // TODO: add more requests with priority carefully

  // ---------------------------------------------------------------------
  // Priority arbitration + Step-3 anti-starvation.
  //
  // Default fold order (highest first): TCM, sourceC, sinkC, sinkD_w,
  // sourceD_w, sourceD_r. Each req sees the OR-union of higher-priority
  // bankSels as its bankSum, and stalls if its own bankSel overlaps.
  //
  // Anti-starvation (only when TCM is present): each cache source runs a
  // saturating counter of consecutive cycles it has been asserting valid
  // without firing. When a source hits WAIT_MAX, we let it "steal" one
  // cycle from TCM by (a) removing tcm.bankSel from that source's bankSum
  // so it fires, and (b) injecting the starved source's bankSel into
  // tcm.bankSum so TCM stalls in exactly the banks it would have taken.
  //
  // Different-stack traffic is untouched: TCM keeps stack X while a
  // starved cache source proceeds on stack Y. Only same-stack conflicts
  // are re-arbitrated.
  // ---------------------------------------------------------------------

  private val StarvationWaitMax = 16
  private val cacheAddrPorts = Seq(
    io.sourceC_raddr, io.sinkC_waddr, io.sinkD_waddr, io.sourceD_waddr, io.sourceD_raddr
  )

  // Per-cache-source saturating stall counters. cnt increments while the
  // port asserts valid but doesn't fire; resets on fire or when valid drops.
  val starvedFlags: Seq[Bool] = cacheAddrPorts.map { port =>
    val cnt = RegInit(0.U(log2Ceil(StarvationWaitMax + 1).W))
    when(port.fire || !port.valid) {
      cnt := 0.U
    }.elsewhen(cnt < StarvationWaitMax.U) {
      cnt := cnt + 1.U
    }
    cnt === StarvationWaitMax.U
  }

  // Precompute default (unadjusted) bankSums for each req from the static
  // priority order — done as a plain Scala fold so we can override.
  private val defaultBankSums: Seq[UInt] = {
    val builder = collection.mutable.ArrayBuffer[UInt]()
    var running: UInt = 0.U(nrBanks.W)
    reqs.foreach { r =>
      builder += running
      running = running | r.bankSel
    }
    builder.toSeq
  }

  if (tcmEnabled) {
    val tcm = tcm_req_opt.get
    // Which banks are being "stolen" from TCM this cycle by starved cache.
    val starvedClaim = cacheReqs.zip(starvedFlags).map {
      case (r, s) => Mux(s, r.bankSel, 0.U(nrBanks.W))
    }.reduce(_ | _)

    // *** DIAGNOSTIC: force anti-starvation off ***
    // Set to `true.B` to disable Step 3 anti-starvation entirely (TCM sees
    // an empty bankSum as if it were still the highest-priority source, and
    // cache reqs see the default fold). If matmul passes with this toggled
    // on, Step 3 is the culprit for the mt>=1 (or nt>=1) failures.
    val disableAntiStarve = true.B

    // TCM sees the stolen banks as if a higher-priority source claimed
    // them, so TCM stalls exactly on those banks.
    tcm.bankSum := Mux(disableAntiStarve, 0.U(nrBanks.W), starvedClaim)

    // Cache sources see their default bankSum, but starved ones exclude
    // TCM's bankSel so they can slip past it.
    cacheReqs.zip(starvedFlags).zipWithIndex.foreach {
      case ((r, s), i) =>
        // reqs = [tcm, cache0, cache1, ...], so cache i is at index i+1.
        val defaultSum = defaultBankSums(i + 1)
        r.bankSum := Mux(disableAntiStarve, defaultSum,
                         Mux(s, defaultSum & ~tcm.bankSel, defaultSum))
    }
  } else {
    // No TCM: plain fold assignments.
    reqs.zip(defaultBankSums).foreach { case (r, s) => r.bankSum := s }
  }

  // Debug perf counters — count how often anti-starvation kicks in.
  if (tcmEnabled) {
    starvedFlags.zip(cacheAddrPorts).zipWithIndex.foreach {
      case ((s, port), i) =>
        XSPerfAccumulate(s"DS_starve_promote_$i", s && port.valid && port.ready)
    }
  }

  val outData = Wire(Vec(nrBanks, UInt((8 * bankBytes).W)))
  val eccData = if (eccBits > 0) Some(Wire(Vec(nrStacks, Vec(stackSize, UInt(eccBits.W))))) else None
  val bank_en = Wire(Vec(nrBanks, Bool()))
  val sel_req = Wire(Vec(nrBanks, new DSRequest))
  dontTouch(bank_en)
  dontTouch(sel_req)

  val cycleCnt = Counter(true.B, 2)
  // mark accessed banks as busy
  if (cacheParams.sramClkDivBy2) {
    bank_en.grouped(stackSize).toList
      .map(banks => Cat(banks).orR)
      .zip(stackRdy)
      .foreach {
        case (accessed, rdy) => rdy := !accessed && cycleCnt._1(0)
      }
  }

  for (i <- 0 until nrBanks) {
    val en = reqs.map(_.bankEn(i)).reduce(_ || _)
    val selectedReq = PriorityMux(reqs.map(_.bankSel(i)), reqs)
    bank_en(i) := en
    sel_req(i) := selectedReq
    if (cacheParams.sramClkDivBy2) {
      // Write — gated additionally by the per-bank write mask so TCM partial
      // writes (byte-lane mask from TL) leave untouched banks intact.
      val wen = en && selectedReq.wen && selectedReq.bankWMask(i)
      val wen_latch = RegNext(wen, false.B)
      bankedData(i).io.w.req.valid := wen_latch
      bankedData(i).io.w.req.bits.apply(
        setIdx = RegNext(selectedReq.index),
        data = RegNext(selectedReq.data(i)),
        waymask = 1.U
      )
      // Read
      val ren = en && !selectedReq.wen
      val ren_latch = RegNext(ren, false.B)
      bankedData(i).io.r.req.valid := ren_latch
      bankedData(i).io.r.req.bits.apply(setIdx = RegNext(selectedReq.index))
    } else {
      // Write — gated by bankWMask for partial-beat TCM writes.
      val wen = en && selectedReq.wen && selectedReq.bankWMask(i)
      bankedData(i).io.w.req.valid := wen
      bankedData(i).io.w.req.bits.apply(
        setIdx = selectedReq.index,
        data = selectedReq.data(i),
        waymask = 1.U
      )
      // Read
      val ren = en && !selectedReq.wen
      bankedData(i).io.r.req.valid := ren
      bankedData(i).io.r.req.bits.apply(setIdx = selectedReq.index)
    }
    // Ecc
    outData(i) := bankedData(i).io.r.resp.data(0)
  }

  if (eccBits > 0) {
    for (((banks, ecc), eccArray) <-
           bankedData.grouped(stackSize).toList
             .zip(eccData.get)
             .zip(dataEccArray)
         ) {
      eccArray.io.w.req.valid := banks.head.io.w.req.valid
      eccArray.io.w.req.bits.apply(
        setIdx = banks.head.io.w.req.bits.setIdx,
        data = VecInit(banks.map(b =>
          dataCode.encode(b.io.w.req.bits.data(0)).head(eccBits)
        )).asUInt,
        waymask = 1.U
      )
      eccArray.io.r.req.valid := banks.head.io.r.req.valid
      eccArray.io.r.req.bits.apply(setIdx = banks.head.io.r.req.bits.setIdx)
      ecc := eccArray.io.r.resp.data(0).asTypeOf(Vec(stackSize, UInt(eccBits.W)))
    }
  } else {
  }

  // DataSel output ports:
  //   0: sourceD read data
  //   1: sourceC read data
  //   2: (optional) TCM read data — only present when tcmEnabled
  private val dataSelOutNum = if (tcmEnabled) 3 else 2
  val dataSelModules = Array.fill(stackSize) {
    Module(new DataSel(nrStacks, dataSelOutNum, bankBytes * 8, eccBits))
  }
  val data_grps = outData.grouped(stackSize).toList.transpose
  val ecc_grps = eccData.map(_.toList.transpose)
  val d_sel = sourceD_rreq.bankEn.asBools.grouped(stackSize).toList.transpose
  val c_sel = sourceC_req.bankEn.asBools.grouped(stackSize).toList.transpose
  val t_sel = tcm_req_opt.map(_.bankEn.asBools.grouped(stackSize).toList.transpose)
  for (i <- 0 until stackSize) {
    val dataSel = dataSelModules(i)
    dataSel.io.in := VecInit(data_grps(i))
    dataSel.io.ecc_in.map(_ := ecc_grps.get(i))
    dataSel.io.sel(0) := Cat(d_sel(i).reverse)
    dataSel.io.sel(1) := Cat(c_sel(i).reverse)
    dataSel.io.en(0) := io.sourceD_raddr.fire
    dataSel.io.en(1) := io.sourceC_raddr.fire
    if (tcmEnabled) {
      dataSel.io.sel(2) := Cat(t_sel.get(i).reverse)
      // Only reads consume the DataSel read pipeline
      dataSel.io.en(2)  := io.tcm_req.get.fire && !io.tcm_req.get.bits.wen
    }
  }

  io.sourceD_rdata.data := Cat(dataSelModules.map(_.io.out(0)).reverse.toIndexedSeq)
  io.sourceD_rdata.corrupt := Cat(dataSelModules.map(_.io.err_out(0)).toIndexedSeq).orR
  io.sourceC_rdata.data := Cat(dataSelModules.map(_.io.out(1)).reverse.toIndexedSeq)
  io.sourceC_rdata.corrupt := Cat(dataSelModules.map(_.io.err_out(1)).toIndexedSeq).orR
  if (tcmEnabled) {
    io.tcm_rdata.get := Cat(dataSelModules.map(_.io.out(2)).reverse.toIndexedSeq)
  }

  val d_addr_reg = RegNextN(io.sourceD_raddr.bits, sramLatency)
  val c_addr_reg = RegNextN(io.sourceC_raddr.bits, sramLatency)

  io.ecc.valid := io.sourceD_rdata.corrupt || io.sourceC_rdata.corrupt
  io.ecc.bits.errCode := EccInfo.ERR_DATA
  io.ecc.bits.addr := Mux(io.sourceD_rdata.corrupt,
    Cat(d_addr_reg.set, d_addr_reg.way, d_addr_reg.beat),
    Cat(c_addr_reg.set, c_addr_reg.way, c_addr_reg.beat)
  )

  val debug_stack_used = PopCount(bank_en.grouped(stackSize).toList.map(seq => Cat(seq).orR))

  for (i <- 1 to nrStacks) {
    XSPerfAccumulate(s"DS_${i}_stacks_used", debug_stack_used === i.U)
  }

}

class DataSel(inNum: Int, outNum: Int, width: Int, eccBits: Int)(implicit p: Parameters) extends HuanCunModule {

  val io = IO(new Bundle() {
    val ecc_in = if (eccBits > 0) Some(Input(Vec(inNum, UInt(eccBits.W)))) else None
    val in = Input(Vec(inNum, UInt(width.W)))
    val sel = Input(Vec(outNum, UInt(inNum.W))) // one-hot sel mask
    val en = Input(Vec(outNum, Bool()))
    val out = Output(Vec(outNum, UInt(width.W)))
    val err_out = Output(Vec(outNum, Bool()))
  })

  def dataCode: Code = Code.fromString(p(HCCacheParamsKey).dataECC)

  for (i <- 0 until outNum) {
    val en = RegNextN(io.en(i), sramLatency - 2)
    val sel_r = RegNextN(io.sel(i), sramLatency - 1)
    val odata = RegEnable(io.in, en)

    io.out(i) := RegEnable(Mux1H(sel_r, odata), RegNext(en, false.B))

    if (eccBits > 0) {
      val oeccs = RegEnable(io.ecc_in.get, en)
      val err = oeccs.zip(odata).map{
        case (e, d) => dataCode.decode(e ## d).error
      }
      io.err_out(i) := RegEnable(Mux1H(sel_r, err).orR, false.B, RegNext(en, false.B))
    } else {
      io.err_out(i) := false.B
    }
  }

}
