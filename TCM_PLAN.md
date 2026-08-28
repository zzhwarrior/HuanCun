# HuanCun TCM 改造计划（方案 A：Way 划分 + 专用独立 SRAM）

## 目标

将 HuanCun 现有 256KB 存储空间一分为二：
- **128KB L2 cache**（ways 0~3，保持原有缓存逻辑）
- **128KB TCM**（专用独立 SRAM，供向量/张量单元直接访问）

两部分在物理 SRAM 上完全隔离，同一周期可并发访问，互不阻塞。

---

## 当前状态 vs 目标状态

### 当前 DataStorage（beatBytes=64，256KB）

```
bankedData[0..15]  16个bank，每bank 2048行×8B
                   全部归 cache 使用（ways 0~7）

一次读出：1个stack = 8 banks × 8B = 64B = 512 bits
```

### 目标 DataStorage

```
bankedData[0..15]    16个bank，每bank 1024行×8B  → 128KB cache（ways 0~3）
                     nrStacks=2，偶/奇 set 分别打 stack-0/stack-1（现有逻辑不变）

tcmBankedData[0..7]  8个独立bank，每bank 2048行×8B → 128KB TCM
                     nrTcmStacks=1，每次访问 8个bank 全部同时激活
                     无 stackIdx 选择，无仲裁逻辑

cache 访问 → 只驱动 bankedData（现有 reqs 链）
TCM 访问  → 只驱动 tcmBankedData（独立使能，全bank激活）
两者物理隔离，同周期并发，各自出 512 bits
```

---

## 需要修改的文件（按依赖顺序）

| 步骤 | 文件 | 修改内容 |
|---|---|---|
| 1 | `HCCacheParameters.scala` | 新增 TCM 参数字段 |
| 2 | `DataStorage.scala` | 核心：新增独立 TCM SRAM + TCM 端口 |
| 3 | `noninclusive/Directory.scala` | 限制替换候选 way 范围（仅 0~3）|
| 4 | `Slice.scala` | 新增 TCM 旁路流水线（绕过 MSHR 和 Directory）|
| 5 | `HuanCun.scala` | 暴露 TCM 对外接口（新增 Diplomacy 从节点）|
| 6 | `WithHuanCunL2.scala` | 传入 TCM 参数，更新 cacheWays=4 |

---

## 详细步骤

---

### Step 1  HCCacheParameters.scala — 新增 TCM 参数

```scala
case class HCCacheParameters(
  ...
  cacheWays: Int = 8,          // 实际给 cache 用的 ways 数量（修改为 4）
  tcmWays:   Int = 0,          // 给 TCM 的 ways 数量（4）
  tcmBaseAddr: Option[BigInt] = None,  // TCM 映射的物理地址起点
)
```

`tcmSize = tcmWays * sets * blockBytes`，由参数推导，无需手填。

---

### Step 2  DataStorage.scala — 核心改造

**2-a  cache SRAM 缩小**

```scala
// 原来：sizeBytes = ways * sets * blockBytes（8 ways）
// 修改：只用 cacheWays
val sizeBytes = cacheParams.cacheWays * cacheParams.sets * cacheParams.blockBytes
// nrRows 自动从 256KB→128KB：2048→1024
```

**2-b  新增 TCM SRAM 实例（8 banks，无 stack 结构）**

```scala
// TCM 参数：取消 stack 交错，全 bank 一次激活
val nrTcmBanks  = 8                              // 固定 8 个 bank
val nrTcmRows   = tcmSizeBytes / (nrTcmBanks * bankBytes)
//              = 128KB / (8 × 8B) = 2048 行

val tcmBankedData = Seq.fill(nrTcmBanks) {
  Module(new SRAMWrapper(
    gen            = UInt((8 * bankBytes).W),     // 每 bank 64 bits
    set            = nrTcmRows,                   // 2048 行
    n              = cacheParams.sramDepthDiv,
    clk_div_by_2   = cacheParams.sramClkDivBy2
  ))
}
```

一次访问 8 × 8B = 64B = **512 bits**，与 cache 的单次读出宽度相同。

**2-c  新增 TCM IO 端口**

```scala
val io = IO(new Bundle() {
  // 现有 cache 端口（不变）
  ...
  // 新增 TCM 端口
  val tcm_raddr = Flipped(DecoupledIO(new TCMAddress))
  val tcm_rdata = Output(new DSData)
  val tcm_waddr = Flipped(DecoupledIO(new TCMAddress))
  val tcm_wdata = Input(new DSData)
})
```

**2-d  TCM 请求处理——无 stack 选择，无仲裁**

cache 的 `req()` 函数需要 `stackIdx`、`accessVec`、`bankSum` 这套仲裁逻辑，
TCM **完全不需要**，控制逻辑如下：

```scala
// TCM 地址解码：物理地址偏移 → SRAM 行号
// tcm_offset 低 6 位为 64B 行内字节偏移（忽略）
// tcm_offset 高位为行号
val tcmRow = io.tcm_raddr.bits.offset >> log2Ceil(nrTcmBanks * bankBytes)
//         = tcm_offset[16:6]，共 11 位，范围 0~2047

// 所有 8 个 bank 同时使能，无选择逻辑
val tcmEn = io.tcm_raddr.valid
for (i <- 0 until nrTcmBanks) {
  tcmBankedData(i).io.r.req.valid        := tcmEn
  tcmBankedData(i).io.r.req.bits.setIdx := tcmRow
}
io.tcm_raddr.ready := true.B  // 单端口单请求，永远 ready

// 读出数据拼接：8 banks → 512 bits
io.tcm_rdata.data := Cat(tcmBankedData.map(_.io.r.resp.data(0)).reverse)
```

**无 bankSum，无 accessVec，无优先级链，无 ready 等待。**
每次请求到达，8 个 bank 立即全部响应。

---

### Step 3  noninclusive/Directory.scala — 限制 way 选择范围

修改 `self_invalid_way_sel`，只在 `ways 0 ~ cacheWays-1` 内选替换目标：

```scala
def self_invalid_way_sel(metaVec: Seq[SelfDirEntry], repl: UInt) = {
  val cacheWays = cacheParams.cacheWays  // = 4
  // 只对前 cacheWays 个 way 做 invalid/trunk 判断
  val invalid_vec = metaVec.zipWithIndex.map {
    case (m, i) => m.state === MetaData.INVALID && i < cacheWays
  }
  ...
}
```

同时 `SubDirectory` 实例化时传入 `ways = cacheWays`（目录 SRAM 随之缩小）。

---

### Step 4  Slice.scala — TCM 旁路流水线

在 SinkA 入口检测地址是否落在 TCM 范围：

```
正常地址 → MSHRAlloc → Directory → MSHR → SourceD（现有路径）

TCM 地址 → TCM bypass 流水线
              │
              ▼
           地址解码（paddr - tcmBase → offset → row = offset[16:6]）
              │
              ▼
           DataStorage.tcm_raddr（8 banks 全激活，无仲裁）
              │  （SRAM 延迟 = sramLatency cycles，固定无抖动）
              ▼
           DataStorage.tcm_rdata（512 bits）
              │
              ▼
           直接拼 TLBundleD（AccessAckData）返回给 tile
```

旁路流水线全程：**无 MSHR 分配、无目录查找、无一致性操作、无仲裁等待**。
延迟 = SRAM 流水线深度（`sramLatency`，与 cache 命中延迟相当，但路径更短）。

---

### Step 5  HuanCun.scala — 对外暴露 TCM

两个可选方向，按复杂度递增：

**方案 5-A（简单）：复用现有 TileLink 从节点**

HuanCun 的 `node` 已经是 TL 从节点。向量单元通过普通 Get/Put 访问 TCM 地址段，
HuanCun 在 Slice 内部按地址分流（Step 4 旁路流水线处理）。
无需新增 Diplomacy 节点，只需在 `AddressSet` 里加入 TCM 地址范围。

**方案 5-B（更灵活）：新增独立 TCM TileLink 从节点**

```scala
val tcmNode = TLManagerNode(Seq(TLSlavePortParameters(
  managers = Seq(TLSlaveParameters(
    address = Seq(AddressSet(tcmBaseAddr, tcmSize - 1)),
    ...
  )),
  beatBytes = beatBytes
)))
```

TCM 节点有独立的 A/D 通道，与 cache 节点完全解耦，物理上可以更宽或提供更多并发。

---

### Step 6  WithHuanCunL2.scala + 配置更新

```scala
val hcParams = HCCacheParameters(
  ...
  cacheWays   = 4,
  tcmWays     = 4,
  tcmBaseAddr = Some(0x60000000L),   // 或其他不冲突地址
)
```

---

## TCM 容量与带宽总结

| 指标 | 数值 | 说明 |
|---|---|---|
| TCM 总容量 | **128KB** | 8 banks × 2048行 × 8B |
| 物理 bank 数 | **8**（独立，与 cache 无关）| 不参与 cache reqs 链 |
| 单次读出宽度 | 8 banks × 8B = 64B = **512 bits** | 全 bank 同时激活 |
| TCM 行数 | **2048 行** | 对应 128KB / 64B |
| stack 数 | **1**（无交错）| 无 stackIdx，无仲裁逻辑 |
| cache 容量（改后）| **128KB** | 4 ways × 512 sets × 64B |
| 同周期并发 | cache 读 + TCM 读 同时各出 512 bits | 物理隔离，**零干扰** |

### cache 与 TCM 的控制逻辑对比

| | cache（bankedData）| TCM（tcmBankedData）|
|---|---|---|
| bank 数 | 16 | **8** |
| stack 数 | 2 | **1** |
| stack 选择 | `stackIdx = set[0]` | **无** |
| 仲裁逻辑 | `bankSum` / `accessVec` 优先级链 | **无** |
| 使能条件 | 按 stack 分时激活 | **8 banks 同时激活** |
| ready 信号 | 需等待 stack 空闲 | **恒为 true** |

---

## 实施顺序与建议

```
Step 1 → Step 2 → Step 3   先让硬件结构建立起来，能仿真
Step 4                      加旁路流水线，跑 TCM 功能测试
Step 5-A                    先用复用节点方式验证端到端
Step 5-B（可选）             如果需要更高并发再升级为独立节点
Step 6                      更新 testL2 config，跑完整仿真
```

每个 Step 都可以独立编译验证，建议按此顺序逐步推进。
