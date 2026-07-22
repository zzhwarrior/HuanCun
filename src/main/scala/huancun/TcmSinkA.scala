/** *************************************************************************************
  * TCM A-channel receiver.
  *
  * Extracted from the shared SinkA/Slice TCM bypass state machine as part of the
  * L2 dual-port refactor. Owns its own put-buffer (no arbitration against cache
  * SinkA) and drives DataStorage.tcm_req directly. Read/Write responses are
  * emitted through a single Decoupled channel that TcmSourceD serialises onto
  * the tcmNode D bus.
  * *************************************************************************************
  */

package huancun

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._

/** Response envelope handed off from TcmSinkA to TcmSourceD. */
class TcmDResp(implicit p: Parameters) extends HuanCunBundle {
  val isRead = Bool()                                       // true: AccessAckData; false: AccessAck
  val source = UInt(sourceIdBits.W)                         // sized to inner edge sourceBits
  val size   = UInt(edgeIn.bundle.sizeBits.W)
  val data   = UInt((nrTcmBanks * 8 * 8).W)                 // full-block data (only meaningful when isRead)
}

/** TCM A-channel handler.
  *
  * @param tcmAddressBase base of the tcmNode address region (used to compute row index)
  */
class TcmSinkA(tcmAddressBase: BigInt)(implicit p: Parameters) extends HuanCunModule {
  val io = IO(new Bundle {
    val a         = Flipped(DecoupledIO(new TLBundleA(edgeIn.bundle)))
    val tcm_req   = DecoupledIO(new TCMReq)
    val resp      = DecoupledIO(new TcmDResp)
    val tcm_rdata = Input(UInt((nrTcmBanks * 8 * 8).W))
  })

  require(tcmEnabled, "TcmSinkA instantiated but tcmEnabled=false")

  // -----------------------------------------------------------------------
  // Per-block put-buffer (private to TcmSinkA — no sharing with cache SinkA)
  // -----------------------------------------------------------------------
  val beats = blockBytes / beatBytes                        // beats per block (e.g. 1 or 8)
  val nrTcmBufBlocks = 2                                    // 2 in-flight puts is plenty for TCM
  val putBuffer = Reg(Vec(nrTcmBufBlocks, Vec(beats, new PutBufferBeatEntry())))
  val beatVals  = RegInit(VecInit(Seq.fill(nrTcmBufBlocks) {
    VecInit(Seq.fill(beats) { false.B })
  }))
  val bufVals   = VecInit(beatVals.map(_.asUInt.orR)).asUInt
  val full      = bufVals.andR
  val insertIdx = PriorityEncoder(~bufVals)
  val insertIdxReg = RegEnable(insertIdx, io.a.fire && edgeIn.first(io.a))
  val hasData   = edgeIn.hasData(io.a.bits)
  val (first, last, _, count) = edgeIn.count(io.a)
  val noSpace   = full && hasData

  // -----------------------------------------------------------------------
  // Row index derivation. tcmNode's address is (tcmAddressBase + offset), so
  // strip the base and shift by log2(rowBytes=nrTcmBanks*8) to get the row.
  // -----------------------------------------------------------------------
  val rowBytesLog2 = log2Ceil(nrTcmBanks * 8)
  def rowOf(addr: UInt): UInt = {
    val off = addr - tcmAddressBase.U(addr.getWidth.W)
    off(off.getWidth - 1, rowBytesLog2)(tcmRowBits - 1, 0)
  }

  // -----------------------------------------------------------------------
  // Write state machine
  // -----------------------------------------------------------------------
  val tcmWrIdle :: tcmWrCollect :: tcmWrCaptureWait :: tcmWrFire :: Nil = Enum(4)
  val tcmWrState   = RegInit(tcmWrIdle)
  val tcmWrBufIdx  = Reg(UInt(log2Ceil(nrTcmBufBlocks).W))
  val tcmWrCount   = RegInit(0.U(log2Ceil(beats + 1).W))
  val tcmWrDataVec = Reg(Vec(beats, UInt((beatBytes * 8).W)))
  val tcmWrRow     = Reg(UInt(tcmRowBits.W))
  val tcmWrSrc     = Reg(UInt(sourceIdBits.W))
  val tcmWrSizeReg = Reg(UInt(edgeIn.bundle.sizeBits.W))

  // -----------------------------------------------------------------------
  // Read pipeline: track in-flight reads through sramLatency stages
  // -----------------------------------------------------------------------
  class TcmInflight extends Bundle {
    val valid  = Bool()
    val source = UInt(sourceIdBits.W)
    val size   = UInt(edgeIn.bundle.sizeBits.W)
  }
  val inflight = RegInit(VecInit(Seq.fill(sramLatency)(0.U.asTypeOf(new TcmInflight))))

  // Only one in-flight read at a time. Since TcmSourceD owns the D channel and
  // always drains via a small Queue, single outstanding is safe/simple.
  val tcmRdOccupied = RegInit(false.B)

  val isRead  = io.a.bits.opcode === TLMessages.Get
  val isWrite = io.a.bits.opcode(2, 1) === 0.U             // PutFull(0) or PutPartial(1)

  val startRead  = io.a.valid && first && isRead && !tcmRdOccupied
  val startWrite = io.a.valid && first && isWrite && (tcmWrState === tcmWrIdle) && !noSpace

  // -----------------------------------------------------------------------
  // A-channel handshake
  // -----------------------------------------------------------------------
  io.a.ready := Mux(hasData,
                    // Write path: accept beats when write SM is idle OR collecting the current block
                    !noSpace && (Mux(first,
                                     tcmWrState === tcmWrIdle,
                                     tcmWrState === tcmWrCollect)),
                    // Read path: accept when pipeline free
                    !tcmRdOccupied)

  // Capture beat data into buffer
  when(io.a.fire && hasData) {
    val idx = Mux(first, insertIdx, insertIdxReg)
    putBuffer(idx)(count).data    := io.a.bits.data
    putBuffer(idx)(count).mask    := io.a.bits.mask
    putBuffer(idx)(count).corrupt := io.a.bits.corrupt
    beatVals(idx)(count) := true.B
  }

  // Start write SM on first beat of a write
  when(startWrite) {
    tcmWrBufIdx  := insertIdx
    tcmWrRow     := rowOf(io.a.bits.address)
    tcmWrSrc     := io.a.bits.source
    tcmWrSizeReg := io.a.bits.size
    tcmWrCount   := 0.U
    tcmWrState   := tcmWrCollect
  }

  // -----------------------------------------------------------------------
  // Write SM: pop beats out of the local put-buffer, capture into DataVec,
  // then fire a single SRAM write.
  // -----------------------------------------------------------------------
  val popLast    = tcmWrCount === (beats - 1).U
  val popValid   = (tcmWrState === tcmWrCollect) && beatVals(tcmWrBufIdx)(tcmWrCount)
  val popCountReg = RegNext(tcmWrCount, 0.U)
  val popFireReg  = RegNext(popValid && (tcmWrState === tcmWrCollect), false.B)

  when(popValid && tcmWrState === tcmWrCollect) {
    tcmWrCount := tcmWrCount + 1.U
    when(popLast) {
      tcmWrState := tcmWrCaptureWait
    }
  }
  // Capture with 1-cycle delay to match the RegEnable in the original design
  when(popFireReg) {
    tcmWrDataVec(popCountReg) := putBuffer(tcmWrBufIdx)(popCountReg).data
  }
  when(tcmWrState === tcmWrCaptureWait) {
    tcmWrState := tcmWrFire
  }
  when(tcmWrState === tcmWrFire) {
    tcmWrState := tcmWrIdle
    beatVals(tcmWrBufIdx).foreach(_ := false.B)             // free the buffer slot
  }

  // -----------------------------------------------------------------------
  // Read handling and in-flight tracker
  // -----------------------------------------------------------------------
  when(startRead) {
    tcmRdOccupied := true.B
  }
  inflight(0).valid  := startRead
  inflight(0).source := io.a.bits.source
  inflight(0).size   := io.a.bits.size
  for (i <- 1 until sramLatency) {
    inflight(i) := inflight(i - 1)
  }

  // -----------------------------------------------------------------------
  // Drive DataStorage.tcm_req
  // -----------------------------------------------------------------------
  io.tcm_req.valid      := (tcmWrState === tcmWrFire) || startRead
  io.tcm_req.bits.wen   := tcmWrState === tcmWrFire
  io.tcm_req.bits.row   := Mux(tcmWrState === tcmWrFire, tcmWrRow, rowOf(io.a.bits.address))
  io.tcm_req.bits.wdata := Cat(tcmWrDataVec.reverse)

  // -----------------------------------------------------------------------
  // Response emission: write ack fires in tcmWrFire; read data emerges from
  // the SRAM sramLatency cycles after startRead — captured on inflight.last.
  // Small skid so slow D-channel drains don't backpressure the SRAM pipeline.
  // -----------------------------------------------------------------------
  val respQ = Module(new Queue(new TcmDResp, entries = 4))
  val rdComplete = inflight.last.valid
  val wrComplete = tcmWrState === tcmWrFire

  // Reads and writes are naturally serialised (only one of tcmRdOccupied /
  // tcmWrFire is true at a time given the accept guards), so a straight
  // Mux is safe; we assert against overlap for sanity.
  assert(!(rdComplete && wrComplete), "TCM read and write responses collided")
  respQ.io.enq.valid       := rdComplete || wrComplete
  respQ.io.enq.bits.isRead := rdComplete
  respQ.io.enq.bits.source := Mux(rdComplete, inflight.last.source, tcmWrSrc)
  respQ.io.enq.bits.size   := Mux(rdComplete, inflight.last.size,   tcmWrSizeReg)
  respQ.io.enq.bits.data   := io.tcm_rdata                  // valid this cycle when rdComplete
  // Free the read pipeline once the response is committed into the queue.
  when(rdComplete && respQ.io.enq.ready) {
    tcmRdOccupied := false.B
  }
  assert(respQ.io.enq.ready || !respQ.io.enq.valid,
    "TcmSinkA response queue overflow -- increase depth")

  io.resp <> respQ.io.deq
}
