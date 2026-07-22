/** *************************************************************************************
  * TCM D-channel driver.
  *
  * Consumes TcmDResp envelopes produced by TcmSinkA and drives the tcmNode D
  * bus. Handles multi-beat AccessAckData bursts when beatBytes < blockBytes.
  * *************************************************************************************
  */

package huancun

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._

class TcmSourceD(implicit p: Parameters) extends HuanCunModule {
  val io = IO(new Bundle {
    val resp = Flipped(DecoupledIO(new TcmDResp))
    val d    = DecoupledIO(new TLBundleD(edgeIn.bundle))
  })

  require(tcmEnabled, "TcmSourceD instantiated but tcmEnabled=false")

  // For AccessAckData bursts: split the block-wide response.data into
  // blockBytes/beatBytes beats. When beatBytes == blockBytes, this reduces
  // to a single beat.
  val beats = blockBytes / beatBytes
  require(beats >= 1)
  val beatIdx = RegInit(0.U(log2Ceil(beats + 1).W))

  val isBurst = io.resp.valid && io.resp.bits.isRead && (beats > 1).B
  val lastBeat = if (beats == 1) true.B else beatIdx === (beats - 1).U

  // Slice the block-wide data by beatIdx. Chisel will constant-fold when beats==1.
  val beatVec = VecInit((0 until beats).map { i =>
    io.resp.bits.data((i + 1) * beatBytes * 8 - 1, i * beatBytes * 8)
  })

  io.d.valid        := io.resp.valid
  io.d.bits         := DontCare
  io.d.bits.opcode  := Mux(io.resp.bits.isRead, TLMessages.AccessAckData, TLMessages.AccessAck)
  io.d.bits.param   := 0.U
  io.d.bits.size    := io.resp.bits.size
  io.d.bits.source  := io.resp.bits.source
  io.d.bits.sink    := 0.U
  io.d.bits.denied  := false.B
  io.d.bits.data    := beatVec(beatIdx)
  io.d.bits.corrupt := false.B

  // Advance beat counter for bursts; dequeue only on last beat.
  when(io.d.fire) {
    when(isBurst && !lastBeat) {
      beatIdx := beatIdx + 1.U
    }.otherwise {
      beatIdx := 0.U
    }
  }
  io.resp.ready := io.d.ready && (!isBurst || lastBeat)
}
