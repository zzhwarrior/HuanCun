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
  val data   = UInt((beatBytes * 8).W)                      // one beat = one stack (see DataStorage refactor)
}

/** TCM A-channel handler.
  *
  * @param tcmAddressBase base of the tcmNode address region (used to compute (way, set))
  */
class TcmSinkA(tcmAddressBase: BigInt)(implicit p: Parameters) extends HuanCunModule {
  val io = IO(new Bundle {
    val a         = Flipped(DecoupledIO(new TLBundleA(edgeIn.bundle)))
    val tcm_req   = DecoupledIO(new TCMReq)
    val resp      = DecoupledIO(new TcmDResp)
    val tcm_rdata = Input(UInt((beatBytes * 8).W))
    // Runtime TCM partition — bit i set ⇒ way i is TCM. We derive the
    // active tcm_way_count and base way index from this via popcount.
    val tcm_way_mask = Input(UInt(effectiveCacheWays.W))
  })

  require(tcmEnabled, "TcmSinkA instantiated but tcmEnabled=false")

  // In our configuration blockBytes == beatBytes, so a Put arrives in a
  // single beat and there is no beat-collect phase to speak of. Simplify
  // the write path aggressively for this case; retain a require() so a
  // future multi-beat config fails loudly.
  val beats = blockBytes / beatBytes
  require(beats == 1,
    s"TcmSinkA fast-path assumes blockBytes==beatBytes (beats=1, got $beats). " +
    "Multi-beat writes need the old put-buffer FSM back.")
  val hasData   = edgeIn.hasData(io.a.bits)
  val (first, last, _, count) = edgeIn.count(io.a)

  // -----------------------------------------------------------------------
  // Address translation: (paddr - tcmAddressBase) → (way, set).
  //
  // TCM occupies the contiguous top tcm_way_count ways (Step 2A: runtime).
  // The MMIO tcmNode's AddressSet is the *maximum* window (all ways possibly
  // TCM); at any moment only the low tcm_way_count * (sets*blockBytes) bytes
  // are actually backed. Accesses beyond that range are DENIED via the
  // out-of-range check below.
  //
  // Convention: "grow-from-top". TCM offset 0 always maps to the highest
  // way (ways-1). Increasing offset descends way indices. This way SW's
  // data at low TCM offsets stays physically at the same location when
  // partition size changes — shrinking TCM simply invalidates the top of
  // the aperture, preserving data below the new cap.
  //
  //   offset[offsetBits-1 : 0]                       = byte within block
  //   offset[offsetBits+setBits-1 : offsetBits]      = set index
  //   offset[offsetBits+setBits+maxTcmWayBits-1 :
  //          offsetBits+setBits]                     = wayInTcmFromTop
  //   actualWay = (ways - 1) - wayInTcmFromTop
  // -----------------------------------------------------------------------
  private val maxTcmWayBits = log2Ceil(effectiveCacheWays) // widest possible slot
  private val activeTcmCount = PopCount(io.tcm_way_mask)   // 0..ways

  def setOf(addr: UInt): UInt = {
    val off = addr - tcmAddressBase.U(addr.getWidth.W)
    off(offsetBits + setBits - 1, offsetBits)
  }
  def wayInTcmOf(addr: UInt): UInt = {
    val off = addr - tcmAddressBase.U(addr.getWidth.W)
    off(offsetBits + setBits + maxTcmWayBits - 1, offsetBits + setBits)
  }
  def wayOf(addr: UInt): UInt = {
    // (ways-1) - wayInTcmFromTop, saturated to wayBits.
    ((effectiveCacheWays - 1).U - wayInTcmOf(addr))(log2Ceil(effectiveCacheWays) - 1, 0)
  }
  // True if the incoming address addresses a way slot beyond the current
  // tcm_way_count — i.e. the access is inside the maximum aperture but
  // outside the currently-enabled TCM. Software should treat this as a bug;
  // we surface it via an assertion (see below).
  def outOfRange(addr: UInt): Bool = wayInTcmOf(addr) >= activeTcmCount

  // -----------------------------------------------------------------------
  // Write path (single-slot pipeline).
  //
  // Since beats==1, the beat data arrives in the same cycle we accept the
  // Put. We latch (way, set, data, mask, source, size) into a "pending"
  // register set and drive io.tcm_req the next cycle. When the SRAM port
  // is free (io.tcm_req.ready), the pending write fires. Critically, we
  // allow accepting a *new* Put in the same cycle we fire the previous —
  // so at steady state throughput is 1 Put per cycle.
  // -----------------------------------------------------------------------
  val pendingWrite  = RegInit(false.B)
  val wrDataReg     = Reg(UInt((beatBytes * 8).W))
  val wrMaskReg     = Reg(UInt(beatBytes.W))
  val wrWayReg      = Reg(UInt(wayBits.W))
  val wrSetReg      = Reg(UInt(setBits.W))
  val wrSrcReg      = Reg(UInt(sourceIdBits.W))
  val wrSizeReg     = Reg(UInt(edgeIn.bundle.sizeBits.W))

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

  // -----------------------------------------------------------------------
  // Write path: single-slot pipeline. Reads have PRIORITY on tcm_req to
  // preserve the double-buffered overlap between AME's tile store (Puts)
  // and the K-loop loads (Gets) of the next tile. A pending write yields
  // one cycle whenever a read is starting; pendingWrite persists so the
  // write fires as soon as no read is contesting.
  //
  //   * startRead         : Get accepts this cycle (unconditional wrt write)
  //   * writeCanFire      : Put is pending AND no read wants tcm_req now
  //   * writeFires        : write actually completes handshake this cycle
  //   * writeSlotFree     : slot free (empty, or firing this cycle)
  // -----------------------------------------------------------------------
  val startRead     = io.a.valid && first && isRead && !tcmRdOccupied
  val writeCanFire  = pendingWrite && !startRead
  val writeFires    = writeCanFire && io.tcm_req.ready
  val writeSlotFree = !pendingWrite || writeFires

  val acceptWrite = io.a.valid && first && isWrite && hasData && writeSlotFree

  // -----------------------------------------------------------------------
  // A-channel handshake
  //   * Read: only gated by the read pipeline being free. Writes never
  //     back-pressure reads at the A channel — this is what protects the
  //     double-buffer overlap.
  //   * Write: still gated by writeSlotFree.
  // -----------------------------------------------------------------------
  io.a.ready := Mux(hasData,
                    writeSlotFree,
                    !tcmRdOccupied)

  // Latch pending write on accept.
  when(acceptWrite) {
    pendingWrite := true.B
    wrDataReg    := io.a.bits.data
    wrMaskReg    := io.a.bits.mask
    wrWayReg     := wayOf(io.a.bits.address)
    wrSetReg     := setOf(io.a.bits.address)
    wrSrcReg     := io.a.bits.source
    wrSizeReg    := io.a.bits.size
  }.elsewhen(writeFires) {
    // Fire without a replacement accept: slot goes empty.
    pendingWrite := false.B
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
  //   * Writes: driven whenever pendingWrite is true, using the latched
  //     (way, set, data, mask). One cycle after A-channel accept.
  //   * Reads:  fire in the same cycle a Get is accepted; DataStorage's
  //     dedicated read pipeline returns tcm_rdata sramLatency cycles later.
  //
  // Writes take priority — same reason as Option 1: we can't drive both
  // wen=1 and wen=0 simultaneously and have the read address stick.
  // -----------------------------------------------------------------------
  // Reduce the per-byte TL mask to a per-8-byte-bank mask. Each bank in the
  // target stack is 8 bytes wide, so bank i is enabled iff any of its 8
  // byte-mask bits is set. Reads present all-ones (unused by DataStorage).
  private val nrBanksPerStack = beatBytes / 8   // = stackSize
  private def reduceMask(m: UInt): UInt = {
    val slices = (0 until nrBanksPerStack).map(i => m(i * 8 + 7, i * 8).orR)
    VecInit(slices).asUInt
  }
  io.tcm_req.valid      := writeCanFire || startRead
  io.tcm_req.bits.wen   := writeCanFire
  io.tcm_req.bits.way   := Mux(writeCanFire, wrWayReg, wayOf(io.a.bits.address))
  io.tcm_req.bits.set   := Mux(writeCanFire, wrSetReg, setOf(io.a.bits.address))
  io.tcm_req.bits.wdata := wrDataReg
  io.tcm_req.bits.wmask := Mux(writeCanFire,
                               reduceMask(wrMaskReg),
                               ~0.U(nrBanksPerStack.W))  // reads: ignored

  // Debug guard: if software addresses a way slot beyond the currently
  // enabled tcm_way_count the resulting (way, set) computation would clobber
  // a cache way's SRAM. Step 2A trusts software to stay within range;
  // Step 2B will replace this assertion with a proper DENIED response.
  assert(!(io.a.valid && first && (isRead || isWrite) && outOfRange(io.a.bits.address)),
    "TcmSinkA: address is inside the max tcmNode window but beyond the current tcm_way_count")

  // -----------------------------------------------------------------------
  // Response emission
  //
  // Writes now fire at up to 1 per cycle (was ~1 per 4 cycles), so we may
  // occasionally collide with a read completion in the same cycle. Priority:
  // read wins (has data), write ack goes into a 1-slot skid register and
  // enqueues on the next available cycle. In DMA streaming there are no
  // reads and the skid never engages.
  // -----------------------------------------------------------------------
  val respQ = Module(new Queue(new TcmDResp, entries = 4))
  val rdComplete = inflight.last.valid
  val wrComplete = writeFires

  val pendingWrAck    = RegInit(false.B)
  val pendingWrAckSrc = Reg(UInt(sourceIdBits.W))
  val pendingWrAckSz  = Reg(UInt(edgeIn.bundle.sizeBits.W))

  // Choose which write-ack payload feeds the queue this cycle: the just-
  // fired one (fresh from wrSrcReg/wrSizeReg) or the skid slot.
  val wrAckReady = wrComplete || pendingWrAck
  val wrAckSrc   = Mux(pendingWrAck, pendingWrAckSrc, wrSrcReg)
  val wrAckSize  = Mux(pendingWrAck, pendingWrAckSz,  wrSizeReg)

  // Read has priority — its data is in flight and can't wait.
  val enqRead  = rdComplete
  val enqWrite = wrAckReady && !enqRead
  respQ.io.enq.valid       := enqRead || enqWrite
  respQ.io.enq.bits.isRead := enqRead
  respQ.io.enq.bits.source := Mux(enqRead, inflight.last.source, wrAckSrc)
  respQ.io.enq.bits.size   := Mux(enqRead, inflight.last.size,   wrAckSize)
  respQ.io.enq.bits.data   := io.tcm_rdata                  // valid this cycle when rdComplete

  // Skid-slot bookkeeping.
  when(wrComplete && enqRead) {
    // Both a fresh write ack and a read completed — read wins, latch write.
    pendingWrAck    := true.B
    pendingWrAckSrc := wrSrcReg
    pendingWrAckSz  := wrSizeReg
    // 3-way conflict (fresh write + read + old pending write) is not
    // reachable: read completes at most once per sramLatency cycles, and
    // during that gap the skid drains.
    assert(!pendingWrAck,
      "TcmSinkA: write-ack skid overflow (3-way response conflict)")
  }.elsewhen(pendingWrAck && enqWrite && respQ.io.enq.ready) {
    // Drained the skid this cycle.
    pendingWrAck := false.B
  }

  // Free the read pipeline once the response is committed into the queue.
  when(rdComplete && respQ.io.enq.ready) {
    tcmRdOccupied := false.B
  }
  assert(respQ.io.enq.ready || !respQ.io.enq.valid,
    "TcmSinkA response queue overflow -- increase depth")

  io.resp <> respQ.io.deq
}
