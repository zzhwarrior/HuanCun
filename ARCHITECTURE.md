# HuanCun Architecture Notes

## 整体结构

```
HuanCun (LazyModule)
└── HuanCunImp
    ├── Slice × nBanks          ← 每个 bank 一个完整的 cache slice
    │   ├── SinkA / SourceA     ← A channel (内→外/外→内 Acquire)
    │   ├── SourceB / SinkB     ← B channel (Probe)
    │   ├── SinkC / SourceC     ← C channel (Release/ProbeAck)
    │   ├── SourceD / SinkD     ← D channel (Grant/GrantData)
    │   ├── SinkE / SourceE     ← E channel (GrantAck)
    │   ├── Directory           ← inclusive / noninclusive 两套实现
    │   ├── MSHR × mshrsAll     ← 每个未完成事务一个 MSHR
    │   ├── MSHRAlloc           ← MSHR 分配与冲突仲裁
    │   ├── RequestBuffer       ← 入口请求缓冲，解决同 set 相关性
    │   ├── DataStorage         ← 多 bank SRAM，带 ECC
    │   └── RefillBuffer        ← Refill bypass buffer，避免绕道 SRAM
    ├── Prefetcher (optional)   ← BOP / SMS / 外部预取接收
    ├── TopDownMonitor (opt.)   ← 性能分析 top-down 支持
    └── CtrlUnit (optional)     ← 测试/调试控制寄存器接口
```

---

## 模块职责速查

### 顶层

| 模块 | 文件 | 职责 |
|---|---|---|
| **HuanCun** | HuanCun.scala | LazyModule 入口；按 bankBits 路由地址到各 Slice；地址拼接/还原 |
| **Slice** | Slice.scala | 单个 bank 的完整 cache 逻辑；Sink/Source/MSHR/Directory 的总协调 |

### TileLink Channel 处理（内侧，面向 Tile）

| 模块 | 通道 | 方向 | 职责 |
|---|---|---|---|
| **SinkA** | A | Tile→L2 | 接收 AcquireBlock/Get/Put；Put 数据写入 PutBuffer；向 MSHRAlloc 发 alloc |
| **SourceB** | B | L2→Tile | 发送 Probe 请求（由 MSHR 驱动） |
| **SinkC** | C | Tile→L2 | 接收 Release/ReleaseData/ProbeAck；脏数据写入 DataStorage |
| **SourceD** | D | L2→Tile | 发送 GrantData（命中走 DataStorage，缺失走 RefillBuffer bypass） |
| **SinkE** | E | Tile→L2 | 接收 GrantAck，通知 MSHR 事务完成 |

### TileLink Channel 处理（外侧，面向 LLC）

| 模块 | 通道 | 方向 | 职责 |
|---|---|---|---|
| **SourceA** | A | L2→LLC | 发送 AcquireBlock（缺失时由 MSHR 驱动）|
| **SinkB** | B | LLC→L2 | 接收来自 LLC 的 Probe |
| **SourceC** | C | L2→LLC | 发送 Release/ProbeAckData（从 DataStorage 读数据）|
| **SinkD** | D | LLC→L2 | 接收 LLC 的 GrantData；写入 DataStorage 或 RefillBuffer |
| **SourceE** | E | L2→LLC | 发送 GrantAck 给 LLC |

### 核心状态与存储

| 模块 | 文件 | 职责 |
|---|---|---|
| **Directory** (inclusive) | inclusive/Directory.scala | 单目录：每个 way 存 tag + 一致性状态（INVALID/BRANCH/TRUNK/TIP） |
| **Directory** (noninclusive) | noninclusive/Directory.scala | 双目录：self dir（自身数据状态）+ client dir（追踪 L1 持有哪些行）|
| **MSHR** | inclusive/MSHR.scala noninclusive/MSHR.scala | 每个进行中的事务一个 MSHR；状态机驱动所有 Source* 任务 |
| **MSHRAlloc** | MSHRAlloc.scala | 给 A/B/C 请求分配 MSHR；发起 directory read；检测 set 冲突和嵌套 |
| **RequestBuffer** | RequestBuffer.scala | A 通道请求入口队列；过滤重复预取；阻塞同 set 相关请求 |
| **DataStorage** | DataStorage.scala | 多 bank SRAMWrapper；各 Source/Sink 按地址仲裁读写端口 |
| **RefillBuffer** | RefillBuffer.scala | SinkD 写入、SourceD bypass 读出；命中时数据不经过 SRAM 直达 Tile |

### 辅助模块

| 模块 | 文件 | 职责 |
|---|---|---|
| **MetaData** | MetaData.scala | 一致性状态常量：INVALID=0, BRANCH=1, TRUNK=2, TIP=3 |
| **Common** | Common.scala | 所有内部消息 Bundle 定义（SinkAReq, SourceDReq, MSHRStatus …）|
| **HCCacheParameters** | HCCacheParameters.scala | 全局参数（sets/ways/mshrs/inclusive 开关 …）和 Diplomacy Key |
| **BankedXbar** | BankedXbar.scala | 多 bank 路由 crossbar（TLCustomNode 实现）|
| **CtrlUnit** | CtrlUnit.scala | MMIO 控制寄存器；支持强制 flush / ECC 注入等测试操作 |
| **TopDownMonitor** | TopDownMonitor.scala | ROB head 地址匹配，判断 L2/L3 miss 是否阻塞提交 |
| **ProbeHelper** (noninclusive) | noninclusive/ProbeHelper.scala | 非 inclusive 模式下管理 Probe 排队与合并 |

---

## 事务数据流

### L2 命中（读）

```
Tile --[A: AcquireBlock]--> SinkA
  → RequestBuffer → MSHRAlloc → Directory (hit, way=W)
  → MSHR 生成 SourceD task
  → DataStorage 读 way W 的数据
  → SourceD --[D: GrantData]--> Tile
  → SinkE 接收 GrantAck
```

### L2 缺失（读）

```
Tile --[A: AcquireBlock]--> SinkA
  → MSHRAlloc → Directory (miss)
  → MSHR 生成 SourceA task
  → SourceA --[A: AcquireBlock]--> LLC
  → SinkD <--[D: GrantData]-- LLC
    ├── 写入 DataStorage（allocate new way）
    └── 写入 RefillBuffer（bypass 路径）
  → MSHR 生成 SourceD task（从 RefillBuffer 读，不等 SRAM）
  → SourceD --[D: GrantData]--> Tile
  → SinkE 接收内侧 GrantAck
  → SourceE --[E: GrantAck]--> LLC
```

### L2 被驱逐（Eviction）

```
MSHR 选择 victim way
  → SourceC 读 DataStorage 并发 Release/ReleaseData → LLC
  → LLC 返回 ReleaseAck（SinkD 处理）
  → 该 way 可被新数据占用
```

---

## 读代码计划

按依赖顺序，由浅入深分 5 个阶段：

### Phase 1 — 数据结构与参数（无依赖，先建立词汇表）
1. `MetaData.scala` — 一致性状态定义，10 分钟
2. `HCCacheParameters.scala` — 所有可配置参数
3. `Common.scala` — 内部消息 Bundle，重点看 MSHRRequest / MSHRStatus / SourceDReq

### Phase 2 — 顶层结构（理解拓扑）
4. `HuanCun.scala` — bank 路由、地址 slice、Slice 实例化
5. `Slice.scala` 头部（只看 IO 定义和子模块实例化，约前 100 行）

### Phase 3 — Channel 处理（了解 TileLink 数据怎么进出）
6. `SinkA.scala` + `SourceD.scala` — 最主要的读路径（A→D）
7. `SinkD.scala` + `RefillBuffer.scala` — 缺失时数据如何从 LLC 流入
8. `SourceA.scala` — 缺失时如何向 LLC 发请求
9. `SinkC.scala`（noninclusive）+ `SourceC.scala` — 写回路径

### Phase 4 — 核心状态机（最复杂，建议结合波形）
10. `BaseDirectory.scala` — 理解读写接口抽象
11. `noninclusive/Directory.scala` — 双目录（self + client）实现
12. `BaseMSHR.scala` — MSHR 接口抽象
13. `noninclusive/MSHR.scala` — 状态机主体，重点看各 state 下生成的 task

### Phase 5 — 仲裁与辅助（理解冲突处理）
14. `MSHRAlloc.scala` — MSHR 分配逻辑，A/B/C 优先级
15. `RequestBuffer.scala` — 同 set 阻塞逻辑
16. `DataStorage.scala` — SRAM bank 仲裁
17. `noninclusive/ProbeHelper.scala` — Probe 合并与排队
