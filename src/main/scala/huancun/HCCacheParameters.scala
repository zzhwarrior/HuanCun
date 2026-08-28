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

import org.chipsalliance.cde.config.Field
import chisel3._
import chisel3.util.{isPow2, log2Ceil}
import freechips.rocketchip.diplomacy.{AddressSet, BufferParams}
import freechips.rocketchip.tilelink.{TLBufferParams, TLChannelBeatBytes, TLEdgeIn, TLEdgeOut}
import freechips.rocketchip.util.{BundleField, BundleFieldBase, BundleKeyBase, ControlKey}
import huancun.prefetch.{PrefetchParameters, TPmetaParameters}
import utility.{MemReqSource, ReqSourceKey}

case object HCCacheParamsKey extends Field[HCCacheParameters](HCCacheParameters())

case class CacheParameters
(
  name: String,
  sets: Int,
  ways: Int,
  blockGranularity: Int,
  blockBytes: Int = 64,
  aliasBitsOpt: Option[Int] = None,
  inner: Seq[CacheParameters] = Nil
) {
  val capacity = sets * ways * blockBytes
  val setBits = log2Ceil(sets)
  val offsetBits = log2Ceil(blockBytes)
  val needResolveAlias = aliasBitsOpt.nonEmpty
}

case object PrefetchKey extends ControlKey[Bool](name = "needHint")

case class PrefetchField() extends BundleField[Bool](PrefetchKey, Output(Bool()), _ := false.B)

case object AliasKey extends ControlKey[UInt]("alias")

case class AliasField(width: Int) extends BundleField[UInt](AliasKey, Output(UInt(width.W)), _ := 0.U(width.W))

// try to keep data in cache is true
// now it only works for non-inclusive cache (ignored in inclusive cache)
case object PreferCacheKey extends ControlKey[Bool](name = "preferCache")

case class PreferCacheField() extends BundleField[Bool](PreferCacheKey, Output(Bool()), _ := false.B)

// indicate whether this block is granted from L3 or not (only used when grantData to L2)
// now it only works for non-inclusive cache (ignored in inclusive cache)
case object IsHitKey extends ControlKey[Bool](name = "isHitInL3")

case class IsHitField() extends BundleField[Bool](IsHitKey, Output(Bool()), _ := true.B)

// indicate whether this block is dirty or not (only used in handle Release/ReleaseData)
// now it only works for non-inclusive cache (ignored in inclusive cache)
case object DirtyKey extends ControlKey[Bool](name = "blockisdirty")

case class DirtyField() extends BundleField[Bool](DirtyKey, Output(Bool()), _ := true.B)

case class CacheCtrl
(
  address: BigInt,
  beatBytes: Int = 8,
  // used to generate core soft reset
  numCores: Int = 1
)

case class HCCacheParameters
(
  name: String = "L2",
  level: Int = 2,
  ways: Int = 4,
  sets: Int = 128,
  blockBytes: Int = 64,
  pageBytes: Int = 4096,
  replacement: String = "plru",
  mshrs: Int = 14,
  dirReadPorts: Int = 1,
  dirReg: Boolean = true,
  enableDebug: Boolean = false,
  enablePerf: Boolean = true,
  hartIds: Seq[Int] = Seq[Int](),
  channelBytes: TLChannelBeatBytes = TLChannelBeatBytes(32),
  prefetch: Option[PrefetchParameters] = None,
  tpmeta: Option[TPmetaParameters] = None,
  elaboratedTopDown: Boolean = true,
  clientCaches: Seq[CacheParameters] = Nil,
  inclusive: Boolean = true,
  alwaysReleaseData: Boolean = false,
  tagECC:            Option[String] = None,
  dataECC:           Option[String] = None,
  echoField: Seq[BundleFieldBase] = Nil,
  reqField: Seq[BundleFieldBase] = Nil, // master
  respKey: Seq[BundleKeyBase] = Nil,
  reqKey: Seq[BundleKeyBase] = Seq(PrefetchKey, PreferCacheKey, AliasKey, ReqSourceKey), // slave
  respField: Seq[BundleFieldBase] = Nil,
  ctrl: Option[CacheCtrl] = None,
  sramClkDivBy2: Boolean = false,
  sramDepthDiv: Int = 1,
  simulation: Boolean = false,
  innerBuf: TLBufferParams = TLBufferParams(),
  outerBuf: TLBufferParams = TLBufferParams(
    a = BufferParams.pipe,
    b = BufferParams.default,
    c = BufferParams.default,
    d = BufferParams.pipe,
    e = BufferParams.default
  ),
  FPGAPlatform: Boolean = false,
  // TCM: when tcmBaseAddr is defined, the top tcmWayCount ways of the unified
  // SRAM pool are dedicated as scratchpad; the remaining ways serve as cache.
  // tcmBaseAddr is the base of the tcmNode's Diplomacy address region.
  tcmBaseAddr: Option[BigInt] = None,
  // Compile-time TCM way count. Must be one of {0, 1, 2, 4, ..., ways} (i.e.,
  // ways or a power-of-2 ≤ ways). Defaults to ways/2 which matches the pre-
  // refactor behaviour of "TCM mirrors the displaced cache half". This value
  // is the *maximum* TCM allocation; a runtime CSR (added in a later step)
  // will let software shrink it below this ceiling.
  tcmWayCountOpt: Option[Int] = None,
  // Base of the MMIO control region for runtime TCM partition (Step 2A).
  // When None, no CSR module is instantiated and the partition is fixed at
  // whatever tcmWayCountOpt resolves to.
  tcmCtrlBaseAddr: Option[BigInt] = None
) {
  require(ways > 0)
  require(sets > 0)
  require(channelBytes.d.get >= 8)
  require(dirReadPorts == 1, "now we only use 1 read port")
  if (!inclusive) {
    require(clientCaches.nonEmpty, "Non-inclusive cache need to know client cache information")
  }
  require(!tcmEnabled || ways % 2 == 0, "ways must be even when TCM is enabled")
  if (tcmEnabled) {
    val n = tcmWayCount
    require(n == 0 || n == ways || isPow2(n),
      s"tcmWayCount ($n) must be 0, a power of 2, or equal to ways ($ways)")
    require(n <= ways,
      s"tcmWayCount ($n) must not exceed ways ($ways)")
    require(isPow2(tcmSizeBytes),
      s"tcmSizeBytes ($tcmSizeBytes) must be a power of 2 for Diplomacy AddressSet")
    require(tcmBaseAddr.get % tcmSizeBytes == 0,
      s"tcmBaseAddr (0x${tcmBaseAddr.get.toString(16)}) must be aligned to tcmSizeBytes ($tcmSizeBytes)")
  }

  def tcmEnabled: Boolean = tcmBaseAddr.isDefined

  // Cache uses all physical ways; some may be masked off at runtime by the
  // tcm_way_mask (see HuanCun.scala). Directory tag SRAMs are always sized
  // for the full 'ways' count so the partition can shift dynamically later.
  def effectiveCacheWays: Int = ways

  // Max TCM way count. Defaults to ways/2 for backwards compatibility with
  // the pre-refactor "half cache half TCM" layout.
  def tcmWayCount: Int = tcmWayCountOpt.getOrElse(if (tcmEnabled) ways / 2 else 0)

  // TCM SRAM capacity — derived from way count, not overridable separately.
  def tcmSizeBytes: Int = tcmWayCount * sets * blockBytes

  // Diplomacy address range served by tcmNode. Always sized to the maximum
  // TCM capacity so the address map stays stable across runtime partition
  // changes (see Step-3 discussion).
  def tcmAddressSet: Option[AddressSet] =
    tcmBaseAddr.map(base => AddressSet(base, tcmSizeBytes - 1))

  def toCacheParams: CacheParameters = CacheParameters(
    name = name,
    sets = sets,
    ways = effectiveCacheWays,
    blockGranularity = log2Ceil(sets),
    blockBytes = blockBytes,
    inner = clientCaches
  )
}

case object EdgeInKey extends Field[TLEdgeIn]

case object EdgeOutKey extends Field[TLEdgeOut]

case object BankBitsKey extends Field[Int]

case object BankIdKey extends Field[Int](0)
