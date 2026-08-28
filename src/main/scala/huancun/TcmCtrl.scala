/** *************************************************************************************
  * TCM partition control — Step 2A.
  *
  * MMIO-mapped register file that lets software dynamically change how many
  * L2 ways are dedicated to the TCM scratchpad. Exposes a way-mask output that
  * is threaded into every Slice so the cache directory (self_invalid_way_sel)
  * and TcmSinkA (address translation) both agree on the current partition.
  *
  * Legal tcm_way_count values: {0, 1, 2, 4, ways}. Writing anything else
  * latches an "illegal write" flag and leaves the mask unchanged.
  *
  * Step 2A caveat: the busy latch is a fixed-length stub — the mask flips
  * instantly. Software is responsible for pre-flushing any cache lines that
  * live in ways about to become TCM (a full L2 wbinvd is the safest option).
  * Step 2B replaces the stub with a real way-flush FSM.
  * *************************************************************************************
  */

package huancun

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper._
import freechips.rocketchip.resources.SimpleDevice
import freechips.rocketchip.tilelink._

case class TcmCtrlParams(
  ctrlAddress:  BigInt,
  ctrlWindow:   BigInt = 0x1000,
  beatBytes:    Int    = 8
) {
  require((ctrlWindow & (ctrlWindow - 1)) == 0 && ctrlWindow >= 0x1000,
    s"ctrlWindow must be a power of 2 and >= 4KB (got 0x${ctrlWindow.toString(16)})")
  require(ctrlAddress % ctrlWindow == 0,
    s"ctrlAddress (0x${ctrlAddress.toString(16)}) must be aligned to ctrlWindow (0x${ctrlWindow.toString(16)})")
}

class TcmCtrl(
  params:          TcmCtrlParams,
  ways:            Int,
  sets:            Int,
  blockBytes:      Int,
  initTcmWayCount: Int,
  busyStubCycles:  Int = 16
)(implicit p: Parameters) extends LazyModule {

  require(ways > 0 && ways <= 32, s"ways ($ways) must be in (0, 32]")
  require(initTcmWayCount == 0 ||
          (initTcmWayCount > 0 && initTcmWayCount < ways &&
           (initTcmWayCount & (initTcmWayCount - 1)) == 0),
    s"initTcmWayCount ($initTcmWayCount) must be 0 or a power of 2 strictly < ways ($ways)")

  val device = new SimpleDevice("tcm-ctrl", Seq("cute,tcm-ctrl"))

  val ctrlNode: TLRegisterNode = TLRegisterNode(
    address     = Seq(AddressSet(params.ctrlAddress, params.ctrlWindow - 1)),
    device      = device,
    beatBytes   = params.beatBytes,
    concurrency = 1
  )

  lazy val module = new TcmCtrlImp(this, ways, sets, blockBytes, initTcmWayCount, busyStubCycles)
}

class TcmCtrlImp(
  outer:           TcmCtrl,
  ways:            Int,
  sets:            Int,
  blockBytes:      Int,
  initTcmWayCount: Int,
  busyStubCycles:  Int
) extends LazyModuleImp(outer) {

  // Contiguous top-N ways form TCM. TCM way_mask ≡ ((1<<count)-1) << (ways-count).
  private def maskFromCount(count: Int): BigInt =
    if (count == 0) BigInt(0)
    else ((BigInt(1) << count) - 1) << (ways - count)

  private val countBits = log2Ceil(ways + 1)
  private val maskBits  = ways

  // ---- Internal state -----------------------------------------------------
  val currentCount = RegInit(initTcmWayCount.U(countBits.W))
  val currentMask  = RegInit(maskFromCount(initTcmWayCount).U(maskBits.W))
  val busy         = RegInit(false.B)
  val illegalWrite = RegInit(false.B)   // latched: last write had illegal value
  val busyTimer    = RegInit(0.U(log2Ceil(busyStubCycles + 1).W))

  // Pending switch (captured on legal write, applied when busy timer expires).
  val pendingCount = RegInit(initTcmWayCount.U(countBits.W))
  val pendingMask  = RegInit(maskFromCount(initTcmWayCount).U(maskBits.W))

  // Busy stub: when a legal write arrives, set busy for busyStubCycles cycles,
  // then commit the pending count/mask. Step 2B replaces this with a real
  // flush FSM whose "done" signal drives the commit.
  when(busy) {
    when(busyTimer === 0.U) {
      currentCount := pendingCount
      currentMask  := pendingMask
      busy         := false.B
    }.otherwise {
      busyTimer := busyTimer - 1.U
    }
  }

  // Exposed status. Legal-write flag is sticky-until-next-legal-write.
  val io = IO(new Bundle {
    // Runtime partition — consumed by HuanCun.slices to drive Directory and
    // TcmSinkA. maskBit(i) = 1 ⇒ way i is TCM.
    val way_mask = Output(UInt(maskBits.W))
    val busy     = Output(Bool())
  })
  io.way_mask := currentMask
  io.busy     := busy

  // ---- Legality check for a write value -----------------------------------
  // Legal values are 0, 1, 2, 4, ..., up to (but not including) ways. The
  // "all-TCM" configuration is excluded in Step 2A because the safeRepl
  // fallback in Directory picks way 0 unconditionally — if that were also
  // TCM, stray cache traffic could clobber TCM data. Step 2B (with a real
  // flush FSM and cache-quiescence check) will relax this.
  def isLegal(v: UInt): Bool = {
    val legalSet =
      (Seq(0) ++ Seq.iterate(1, log2Ceil(ways) + 1)(_ * 2).takeWhile(_ < ways)).distinct
    legalSet.map(x => v === x.U).reduce(_ || _)
  }

  // Static regmap table computes a mask value per legal count for the case
  // statement below. We build `count → mask` at elaboration.
  private val legalCounts: Seq[Int] =
    (Seq(0) ++ Seq.iterate(1, log2Ceil(ways) + 1)(_ * 2).takeWhile(_ < ways)).distinct

  def maskFromCountUInt(count: UInt): UInt = {
    // Convert a UInt count (guaranteed legal) into the corresponding mask.
    MuxLookup(count, 0.U(maskBits.W))(
      legalCounts.map(c => c.U(countBits.W) -> maskFromCount(c).U(maskBits.W))
    )
  }

  // ---- Regmap -------------------------------------------------------------
  //   0x00  TCM_MODE   [3:0] target count (write); [3:0] current count (read)
  //   0x04  TCM_STATUS [0]=busy, [1]=illegal_last_write
  //   0x08  TCM_MASK   [ways-1:0] active way mask
  //   0x0C  TCM_INFO   [3:0]=ways, [15:8]=setsLog2, [23:16]=blockBytesLog2
  val info =
    (BigInt(log2Ceil(blockBytes)) << 16) |
    (BigInt(log2Ceil(sets))       <<  8) |
     BigInt(ways)

  outer.ctrlNode.regmap(
    0x00 -> Seq(RegField(32,
      RegReadFn(currentCount),
      RegWriteFn((valid: Bool, data: UInt) => {
        when(valid && !busy) {
          val v = data(countBits - 1, 0)
          when(isLegal(v)) {
            illegalWrite := false.B
            when(v =/= currentCount) {
              pendingCount := v
              pendingMask  := maskFromCountUInt(v)
              busy         := true.B
              busyTimer    := busyStubCycles.U
            }
          }.otherwise {
            illegalWrite := true.B
          }
        }
        true.B
      }),
      RegFieldDesc("tcm_mode", "TCM way-count target (write) / current (read)"))),

    0x04 -> Seq(RegField.r(32,
      Cat(0.U(30.W), illegalWrite, busy),
      RegFieldDesc("tcm_status", "[0]=busy, [1]=illegal_last_write"))),

    0x08 -> Seq(RegField.r(32,
      currentMask.pad(32),
      RegFieldDesc("tcm_mask", "Active way mask (bit i set ⇒ way i is TCM)"))),

    0x0C -> Seq(RegField.r(32,
      info.U(32.W),
      RegFieldDesc("tcm_info", "[3:0]=ways, [15:8]=log2(sets), [23:16]=log2(blockBytes)")))
  )
}
