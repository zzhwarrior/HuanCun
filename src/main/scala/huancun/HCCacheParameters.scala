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
  // TCM: when tcmBaseAddr is defined, half the SRAM banks become TCM.
  // Since the dual-port refactor, tcmBaseAddr denotes the base address of the
  // dedicated tcmNode's address region, not a filter inside the cache pipeline.
  tcmBaseAddr: Option[BigInt] = None,
  // Explicit TCM capacity (bytes). When None, falls back to the legacy derivation
  // where TCM mirrors the displaced cache half (effectiveCacheWays * sets * blockBytes).
  tcmSizeBytesOpt: Option[Int] = None
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
    require(isPow2(tcmSizeBytes),
      s"tcmSizeBytes ($tcmSizeBytes) must be a power of 2 for Diplomacy AddressSet")
    require(tcmBaseAddr.get % tcmSizeBytes == 0,
      s"tcmBaseAddr (0x${tcmBaseAddr.get.toString(16)}) must be aligned to tcmSizeBytes ($tcmSizeBytes)")
  }

  def tcmEnabled: Boolean = tcmBaseAddr.isDefined

  // When TCM is enabled nrStacks=1, so the cache uses half as many banks and
  // half as many ways to keep the physical SRAM depth unchanged.
  def effectiveCacheWays: Int = if (tcmEnabled) ways / 2 else ways

  // TCM SRAM capacity per bank/slice. Explicit override wins; otherwise mirrors
  // the cache half it displaces.
  def tcmSizeBytes: Int =
    tcmSizeBytesOpt.getOrElse(effectiveCacheWays * sets * blockBytes)

  // Diplomacy address range served by tcmNode.
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
