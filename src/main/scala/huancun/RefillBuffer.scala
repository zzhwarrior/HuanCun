package huancun

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._

class SourceDBufferRead(implicit p: Parameters) extends HuanCunBundle {
  val valid = Input(Bool())
  val beat = Input(UInt(beatBits.W))
  val id = Input(UInt(bufIdxBits.W))
  val ready = Output(Bool())
  val buffer_data = Output(new DSData)
  val last = Input(Bool())
}

class SinkDBufferWrite(implicit p: Parameters) extends HuanCunBundle {
  val valid = Input(Bool())
  val beat = Input(UInt(beatBits.W))
  val data = Input(new DSData)
  val ready = Output(Bool())
  val id = Output(UInt(bufIdxBits.W))
}

/**
  *   RefillBuffer is used to reduce outer grant -> inner grant latency
  *   refill data can be bypassed to inner cache without go through SRAM
  */
class RefillBuffer(implicit p: Parameters) extends HuanCunModule {
  val io = IO(new Bundle() {
    val r = new SourceDBufferRead()
    val w = new SinkDBufferWrite()
  })

  val buffer = Mem(bufBlocks, Vec(beatSize, new DSData()))
  val valids = RegInit(VecInit(Seq.fill(bufBlocks){
    VecInit(Seq.fill(beatSize){false.B})
  }))

  val (r, w) = (io.r, io.w)
  val rlast = r.last
  val wlast = w.beat.andR
  val wfirst = w.beat === 0.U

  // write-forward: if write and read hit same id/beat in same cycle, forward data directly.
  // r.valid must be included so that forward_hit is false when no read is pending — otherwise
  // we would skip the buffer write while the data has not actually been consumed.
  val w_fire = w.valid && w.ready
  val forward_hit = w_fire && r.valid && w.id === r.id && w.beat === r.beat

  r.buffer_data := Mux(forward_hit, w.data, buffer(r.id)(r.beat))
  r.ready := valids(r.id)(r.beat) || forward_hit

  when(r.valid && r.beat === 0.U){
    assert(r.ready, "[%d] first beat must hit!", r.id)
  }

  when(r.valid && r.ready && rlast && !forward_hit){ // last beat, only invalidate if data was from buffer
    valids(r.id).foreach(_ := false.B)
  }

  val validMask = VecInit(valids.map(vec => vec.asUInt.orR)).asUInt
  val freeIdx = PriorityEncoder(~validMask)

  // Use current-cycle freeIdx directly so SinkD resp.bufIdx is correct in the same cycle as wfirst.
  // w.ready is combinational to avoid the one-cycle gap between ready and id.
  val id_reg = RegEnable(freeIdx, w.valid && w.ready && wfirst)
  w.ready := Mux(wfirst, !validMask.andR, true.B)
  w.id := Mux(wfirst, freeIdx, id_reg)

  when(w.valid && w.ready && !forward_hit){
    // Only write to buffer when not forwarding directly to a simultaneous read.
    // When forward_hit is true the data is consumed in the same cycle; writing it
    // to the buffer would leave a stale valid entry that blocks future allocations.
    printf("[RefillBuffer] WRITE: w.id=%d w.beat=%d validMask=%b freeIdx=%d\n", w.id, w.beat, validMask, freeIdx)
    assert(!valids(w.id)(w.beat), "[%d] attempt to write a valid entry", w.id)
    valids(w.id)(w.beat) := true.B
    buffer(w.id)(w.beat) := w.data
  }

}
