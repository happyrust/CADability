# 将 CADability 布尔运算移植到 truck-fork 的分析与方案

## 一、两套系统的架构对比

### 算法流水线对比

| 阶段 | CADability (`BRepIntersection.cs`) | truck-fork (`truck-shapeops`) |
|------|-----------------------------------|-------------------------------|
| **1. 求交候选筛选** | 八叉树（BRepOperation 继承 OctTree） | 网格碰撞（AABB + 三角形相交） |
| **2. 曲面-曲面交线** | 参数域网格扫描 + 牛顿迭代（`BoxedSurfaceEx.Intersect`） | **Mesh-based**：三角剖分 → `extract_interference` → 多段线 |
| **3. Edge-Face 穿刺** | 直接求解曲面-曲线交点（`Face.IntersectAndPosition`） | 通过 mesh 碰撞间接获得 |
| **4. Edge 拆分** | 在穿刺点处拆分 Edge（`splitEdges`） | 由 `loops_store` 在 Wire 上执行 |
| **5. 交线 Edge 创建** | 连接穿刺点构造 `InterpolatedDualSurfaceCurve` | 构造 `IntersectionCurve<BSplineCurve, S, S>` |
| **6. 面裁剪** | 2D 参数空间 FindLoop（最左转策略） | `divide_face`（参数空间多边形面积+包含判断） |
| **7. 面分类** | 拓扑连通性扩展（从裁剪面通过 Edge 传播） | `FacesClassification`（And/Or/Unknown）+ 射线法 |
| **8. 壳体组装** | `extractConnectedFaces` + `Shell.MakeShell` | `connected_components` + `Solid::new` |
| **9. 修补** | `TryConnectOpenEdges`, `TryFixMissingFaces` | 无显式修补（依赖精确计算） |

### 核心差异

```
CADability 方法：精确参数域求交（Exact Parametric）
  Surface₁ × Surface₂ → 参数域网格 → 牛顿精化 → InterpolatedDualSurfaceCurve
  
truck 方法：网格近似求交（Mesh-Approximate）
  Surface₁, Surface₂ → 三角剖分 → 三角形碰撞 → 多段线 → IntersectionCurve(leader + Newton投影)
```

**truck 的根本弱点**：交线精度受限于网格密度。这是 truck 布尔运算在复杂曲面上经常失败（返回 `None`）的根本原因。

**CADability 的根本弱点**：11,000 行单文件代码中大量退化处理（切向、重叠面等），但经过多年实战检验。

---

## 二、truck-fork 的现有能力与缺失

### 已有能力

| 能力 | 状态 | 位置 |
|------|------|------|
| BRep 拓扑结构 | ✅ 完善 | `truck-topology` |
| NURBS 曲线/曲面 | ✅ 完善 | `truck-geometry` |
| 参数曲面求值/求导 | ✅ 完善 | `ParametricSurface` trait |
| Newton 点投影 | ✅ 有 | `SearchParameter`, `SearchNearestParameter` |
| 曲线-曲线交点（单点） | ✅ 有 | `search_intersection_parameter` |
| 曲线-曲面交点（单点） | ✅ 有 | `search_intersection_parameter` |
| IntersectionCurve 表示 | ✅ 有 | `IntersectionCurve<C, S0, S1>` |
| 网格碰撞检测 | ✅ 有 | `extract_interference`, `Collision` |
| and/or 布尔运算 | ✅ 基础可用 | `truck-shapeops::and()`, `or()` |
| Solid::not() | ✅ 有 | 反转所有面的朝向 |
| 面裁剪（divide_face） | ✅ 基础可用 | `divide_faces` |
| STEP I/O | ✅ 有 | `truck-stepio` |

### 关键缺失

| 缺失能力 | 重要性 | CADability 中对应 |
|---------|--------|------------------|
| **参数域曲面-曲面交线（精确）** | 🔴 关键 | `BoxedSurfaceEx.Intersect` |
| **重叠面检测与处理** | 🔴 关键 | `findOverlappingFaces`, `oppositeFaces`, `cancelledfaces` |
| **切向相交处理** | 🔴 关键 | 容差放大、法线采样 |
| **交线-原始边重合检测** | 🟡 重要 | `SameEdge`, 去重逻辑 |
| **非流形 Edge 处理** | 🟡 重要 | `nonManifoldEdges` |
| **开放边修补** | 🟡 重要 | `TryConnectOpenEdges`, `TryFixMissingFaces` |
| **八叉树/BVH 空间索引** | 🟡 重要 | `OctTree<BRepItem>` |
| **差集运算（显式）** | 🟢 简单 | `Solid.Subtract`（= and + not） |
| **平面分割** | 🟢 简单 | `SplitByPlane` |
| **全根曲线-曲线交点** | 🟡 重要 | 多个交点的处理 |
| **闭合交线环检测** | 🟡 重要 | `createInnerFaceIntersections` |

---

## 三、移植路线图

### 路线 A：渐进式增强（推荐）

在 truck 现有 mesh-based 架构上逐步添加退化处理和精确求交能力。

```
阶段 0：差集运算（1 天）
  ↓
阶段 1：增强鲁棒性（1-2 周）
  ↓
阶段 2：精确参数域求交（2-4 周）
  ↓
阶段 3：退化情况全面处理（4-8 周）
```

#### 阶段 0：差集运算

truck 有 `and()` 和 `or()`，差集只需：

```rust
// truck-shapeops/src/transversal/integrate/mod.rs
pub fn subtract<C: ShapeOpsCurve<S>, S: ShapeOpsSurface>(
    solid0: &Solid<Point3, C, S>,
    solid1: &Solid<Point3, C, S>,
    tol: f64,
) -> Option<Solid<Point3, C, S>> {
    let mut neg_solid1 = solid1.clone();
    neg_solid1.not();  // 反转 solid1
    and(solid0, &neg_solid1, tol)  // A - B = A ∩ ¬B
}
```

这与 CADability 的做法完全一致（`BRepIntersection.cs:2147-2150`）。

#### 阶段 1：增强鲁棒性

**1.1 重叠面检测**（移植 CADability 的 `findOverlappingFaces`）

在 `loops_store` 之前添加预处理：

```rust
// 新文件：truck-shapeops/src/transversal/overlapping_faces.rs

pub struct OverlappingFacePair<S> {
    face0_id: FaceID<S>,
    face1_id: FaceID<S>,
    same_orientation: bool,
    uv_transform: Option<Matrix3<f64>>,  // face0 UV → face1 UV
}

pub fn find_overlapping_faces<C, S>(
    shell0: &Shell<Point3, C, S>,
    shell1: &Shell<Point3, C, S>,
    tol: f64,
) -> Vec<OverlappingFacePair<S>>
where
    S: ParametricSurface3D + SearchParameter<D2, Point = Point3>,
{
    // 对每对面：
    // 1. AABB 快速排除
    // 2. 在多个采样点上比较 surface.subs() 和 normal()
    // 3. 如果所有采样点的距离 < tol 且法线同向/反向 → 标记为重叠
    todo!()
}
```

**1.2 切向相交处理**

在 `ShapesOpStatus::from_is_curve` 中添加切向检测：

```rust
// loops_store/mod.rs 中修改
fn from_is_curve<C, S0, S1>(curve: &IntersectionCurve<C, S0, S1>) -> Option<ShapesOpStatus> {
    let (t0, t1) = curve.range_tuple();
    let t = (t0 + t1) / 2.0;
    let (_, pt0, pt1) = curve.search_triple(t, 100)?;
    let der = curve.leader().der(t);
    let normal0 = curve.surface0().normal(pt0[0], pt0[1]);
    let normal1 = curve.surface1().normal(pt1[0], pt1[1]);
    let cross = normal0.cross(der);
    
    // 新增：切向检测
    if cross.magnitude() < 1e-4 {
        // 曲面几乎切向，需要更精细的判断
        // 采样偏移点来确定方向
        return determine_tangential_status(curve, t);
    }
    
    match cross.dot(normal1) > 0.0 {
        true => Some(ShapesOpStatus::Or),
        false => Some(ShapesOpStatus::And),
    }
}
```

**1.3 空间索引加速**

添加 BVH 加速三角形碰撞检测：

```rust
// truck-polymesh 或 truck-base 中添加
// 可以使用 Rust 的 bvh crate
```

#### 阶段 2：精确参数域曲面-曲面交线

这是最核心的移植内容。将 CADability 的 `BoxedSurfaceEx.Intersect` 算法翻译为 Rust：

```rust
// 新文件：truck-geometry/src/algo/surface_surface_intersection.rs

/// 精确参数域曲面-曲面交线计算
pub fn intersect_surfaces<S0, S1>(
    surface0: &S0,
    bounds0: (RangeInclusive<f64>, RangeInclusive<f64>),
    surface1: &S1,
    bounds1: (RangeInclusive<f64>, RangeInclusive<f64>),
    tol: f64,
) -> Vec<IntersectionCurve<BSplineCurve<Point3>, S0, S1>>
where
    S0: ParametricSurface3D,
    S1: ParametricSurface3D,
{
    // CADability 算法移植：
    // 1. 将两个参数域分为网格（等参线）
    // 2. 对每条等参线与另一个曲面求交
    // 3. 得到 LinkedIntersectionPoint 集合
    // 4. 链接为有序曲线
    // 5. 构造 IntersectionCurve
    todo!()
}
```

关键数据结构：

```rust
struct LinkedIntersectionPoint {
    point: Point3,
    uv0: Point2,        // 在 surface0 上的参数
    uv1: Point2,        // 在 surface1 上的参数
    cross: Vector3,     // normal0 × normal1（交线方向）
    grid_index: GridIndex,
    // 连接信息
    prev: Option<usize>,
    next: Option<usize>,
}

enum GridIndex {
    FixedU0 { u_idx: usize, v_range: (usize, usize) },
    FixedV0 { v_idx: usize, u_range: (usize, usize) },
    FixedU1 { u_idx: usize, v_range: (usize, usize) },
    FixedV1 { v_idx: usize, u_range: (usize, usize) },
}
```

#### 阶段 3：退化情况全面处理

按 `BooleanOperationDegenerateCases.md` 中的分类逐一实现：

| 优先级 | 退化类别 | 实现要点 |
|--------|---------|---------|
| P0 | B 重叠面 | `find_overlapping_faces` + 公共部分提取 |
| P0 | A 切向 | 容差自适应 + 偏移采样法线 |
| P1 | C 交线-边重合 | `same_edge` 几何比较 + 去重 |
| P1 | F 包含关系 | 射线法已有，完善空壳判断 |
| P1 | E 极点/接缝 | 周期面预处理（truck 已有 `SplitClosedEdgesAndFaces`） |
| P2 | D 非流形 | 非流形 Edge 标记与隔离 |
| P2 | G 环歧义 | 最左转策略 |
| P2 | H 曲面交线退化 | 网格细分重试机制 |
| P3 | I 修补 | 开放边连接 + 缺失面填充 |

---

### 路线 B：替换核心求交引擎

直接将 CADability 的 `BRepOperation` 算法完整移植为 Rust 实现，替换 truck 的 mesh-based 方法。

**优点**：获得经过实战检验的算法，包括所有退化处理

**缺点**：
- 工作量巨大（11,000+ 行 C# → Rust）
- 需要适配 truck 的泛型拓扑系统（CADability 用具体类型，truck 用泛型 `<P, C, S>`）
- CADability 的调试代码（`#if DEBUG`）占约 30% 代码量

---

## 四、具体实现指南

### 4.1 类型映射

| CADability | truck-fork | 说明 |
|-----------|-----------|------|
| `GeoPoint` / `GeoPoint2D` | `Point3` / `Point2` | cgmath |
| `GeoVector` / `GeoVector2D` | `Vector3` / `Vector2` | cgmath |
| `Vertex` | `Vertex<Point3>` | 带 `Arc<Mutex>` |
| `Edge` | `Edge<Point3, C>` | truck 用泛型曲线 |
| `Face` | `Face<Point3, C, S>` | truck 用泛型曲面 |
| `Shell` | `Shell<Point3, C, S>` | |
| `Solid` | `Solid<Point3, C, S>` | |
| `ISurface` | `ParametricSurface3D` trait | |
| `ICurve` / `ICurve2D` | `ParametricCurve3D` / `ParametricCurve<Point = Point2>` | |
| `InterpolatedDualSurfaceCurve` | `IntersectionCurve<C, S0, S1>` | 概念相同 |
| `BRepItem` (OctTree 载荷) | 无等价物（需新建） | |
| `OctTree<BRepItem>` | 无（需引入 BVH/八叉树） | |
| `Border` / `SimpleShape` | `Wire<P, C>` 的 2D 边界表示 | |
| `BRepOperation.Operation` | `ShapesOpStatus` (And/Or) | CADability 用 enum |
| `Precision.eps` | `TOLERANCE` (1e-6) | |

### 4.2 关键算法翻译示例

**CADability 的 `findOverlappingFaces` → Rust：**

```rust
use truck_topology::*;
use truck_base::cgmath64::*;

pub struct OverlappingPair<S> {
    pub face0: FaceID<S>,
    pub face1: FaceID<S>,
    pub same_orientation: bool,
}

pub fn find_overlapping_faces<P, C, S>(
    shell0: &Shell<P, C, S>,
    shell1: &Shell<P, C, S>,
    tol: f64,
) -> Vec<OverlappingPair<S>>
where
    P: Copy + EuclideanSpace<Scalar = f64>,
    S: ParametricSurface3D + SearchParameter<D2, Point = Point3> + Bounded<Point3>,
{
    let mut result = Vec::new();
    
    for face0 in shell0.face_iter() {
        let bb0 = face0.surface().bounding_box(); // 需要 Bounded trait
        for face1 in shell1.face_iter() {
            let bb1 = face1.surface().bounding_box();
            
            // AABB 快速排除
            if bb0.disjoint(&bb1) { continue; }
            
            // 采样点比较
            let surface0 = face0.surface();
            let surface1 = face1.surface();
            let (u_range, v_range) = surface0.parameter_range();
            
            let mut is_same_geometry = true;
            let mut is_same_orient = true;
            const N: usize = 5;
            
            for i in 0..=N {
                for j in 0..=N {
                    let u = u_range.0 + (u_range.1 - u_range.0) * i as f64 / N as f64;
                    let v = v_range.0 + (v_range.1 - v_range.0) * j as f64 / N as f64;
                    let pt = surface0.subs(u, v);
                    
                    // 尝试将点投影到 surface1
                    if let Some((u1, v1)) = surface1.search_parameter(pt, None, 100) {
                        let pt1 = surface1.subs(u1, v1);
                        if pt.distance(pt1) > tol {
                            is_same_geometry = false;
                            break;
                        }
                        let n0 = surface0.normal(u, v);
                        let n1 = surface1.normal(u1, v1);
                        if n0.dot(n1) < 0.0 {
                            is_same_orient = false;
                        }
                    } else {
                        is_same_geometry = false;
                        break;
                    }
                }
                if !is_same_geometry { break; }
            }
            
            if is_same_geometry {
                result.push(OverlappingPair {
                    face0: face0.id(),
                    face1: face1.id(),
                    same_orientation: is_same_orient,
                });
            }
        }
    }
    result
}
```

### 4.3 truck 特有的设计考量

**泛型系统**：truck 的拓扑层完全泛型化（`Shell<P, C, S>`），布尔运算通过 trait bound 约束。新增的算法必须通过 trait 实现，不能假设具体类型。

**Arc<Mutex> 共享语义**：truck 的顶点、边、面通过 `Arc<Mutex>` 共享。修改一个 Edge 的曲线会影响所有引用它的 Face。这与 CADability 的克隆策略不同——CADability 在布尔运算开始时就克隆了整个 Shell。

```rust
// truck 中修改边需要：
let edge_curve = edge.curve();  // 返回 Arc<Mutex<C>>
let mut curve = edge_curve.lock().unwrap();
// 修改 curve...
// 所有引用此 Edge 的 Face 自动看到变化
```

**Option 返回值**：truck 的布尔运算返回 `Option<Solid>`，而非 panic。这是因为 mesh-based 方法可能在退化情况下失败。移植 CADability 的退化处理可以大幅减少 `None` 返回。

---

## 五、工作量估算

| 阶段 | 工作量 | 产出 |
|------|--------|------|
| 阶段 0：差集运算 | 1 天 | `subtract()` 函数 |
| 阶段 1.1：重叠面检测 | 3-5 天 | `find_overlapping_faces` + 集成 |
| 阶段 1.2：切向处理 | 2-3 天 | 修改 `from_is_curve` + 容差调整 |
| 阶段 1.3：BVH 加速 | 2-3 天 | 引入 `bvh` crate |
| 阶段 2：精确 SSI | 2-4 周 | `intersect_surfaces` + 测试 |
| 阶段 3：全面退化处理 | 4-8 周 | 30+ 子类退化处理 |
| **总计** | **8-16 周** | 工业级布尔运算引擎 |

---

## 六、推荐策略

**短期（1-2 周）**：实现阶段 0 + 阶段 1.1（差集 + 重叠面），这会立即解决 truck 最常见的失败案例。

**中期（1-2 月）**：实现阶段 2（精确 SSI），这是从"能用"到"好用"的关键跃升。可以让 mesh-based 和 parametric 两条路径共存，parametric 作为 mesh 方法失败时的 fallback。

**长期（2-4 月）**：阶段 3 的退化处理。这些是 CADability 用了多年才积累下来的 edge case，可以参考 `BooleanOperationDegenerateCases.md` 逐项移植。

**混合策略**（推荐）：
```
truck 的 mesh-based 方法（快速路径）
    ↓ 如果返回 None
CADability 的 parametric 方法（精确路径）
    ↓ 如果仍然失败
返回 None 并记录诊断信息
```

这种分层策略可以在保持 truck 速度优势的同时获得 CADability 的鲁棒性。
