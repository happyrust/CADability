# Truck 参数域精确布尔运算开发计划

## 代码位置

- **分支**: `feature/parametric-boolean-ops` (在 `happyrust/CADability` 仓库)
- **源码**: `truck-shapeops/src/transversal/parametric/`
- **测试**: `truck-shapeops/tests/boolean_comparison.rs`

## 已完成

### Phase 1: 核心参数域 SSI ✅
- `surface_surface_intersection.rs`: Newton 等参线扫描 + 4×4 最小二乘精化
- `edge_face_intersection.rs`: Newton Edge-Face 穿刺检测
- `face_splitter.rs`: 2D 参数空间面裁剪
- 测试: 球-球交线 (r=1 at z=0), 抛物面交线 (4.5×10⁻⁶ 误差)

### Phase 2: 完整流水线集成 ✅
- `integrate_parametric.rs`: `and_parametric()`, `or_parametric()`, `subtract_parametric()`
- `create_loops_stores_parametric()`: 替换 mesh `extract_interference`
- `estimate_surface_bounds()`: 从顶点推算 UV 范围
- JSON 双向对比测试框架: mesh vs parametric 结果比较

### Phase 3: 解析平面交线 ✅
- `analytic_intersection.rs`: 平面-平面解析交线, 平面-曲面有符号距离二分
- `intersect_surfaces()`: 统一 SSI 分发器 (Plane×Plane / Plane×Curved / Curved×Curved)
- `is_planar()`: 平面性检测

## 待完成

### Phase 4: IntersectionCurve 投影修复 (关键瓶颈) 🔴
**问题**: `IntersectionCurveWithParameters::try_new()` 内部调用 `search_triple()` 做 Newton 双投影, 但对于已精确在两平面上的点, 投影退化。

**方案**:
1. 创建 `IntersectionCurveWithParameters::new_from_known_params()` — 直接从已知 UV 坐标构建, 绕过 Newton 投影
2. 在 `analytic_intersection.rs` 中用解析公式直接计算两个 surface 的 UV 坐标
3. 测试: cube×cube AND/OR/SUBTRACT 应与 mesh 方法得到相同拓扑

### Phase 5: 重叠面检测 🟡
**问题**: 两个 Shell 的 Face 共享相同几何曲面时, SSI 找不到交线

**方案**:
1. 在 `create_loops_stores_parametric` 前检测重叠面对
2. 同向重叠 → 提取公共部分
3. 反向重叠 → 标记为抵消面
4. 参考 CADability `findOverlappingFaces()`

### Phase 6: 切向处理 🟡
**问题**: 切向 Edge-Face 交点精度差, 切向面法线判断失效

**方案**:
1. 检测 `edge.direction · face.normal < threshold`
2. 放大容差 (参考 CADability `Precision.eps * 1000`)
3. 从面内部偏移采样法线方向

### Phase 7: 非流形和修补 🟢
- 非流形 Edge 检测和隔离
- 开放边连接 (`TryConnectOpenEdges`)
- 缺失面填充

### Phase 8: AABB 加速 🟢
- 面对级别 AABB 快速排除 (跳过不相交的面对)
- 减少无效 SSI 计算

## 测试矩阵

| 测试类别 | mesh 方法 | parametric 方法 | 状态 |
|---------|----------|----------------|------|
| cube×cube AND | ✅ 6 faces | ⚠️ 0 faces | Phase 4 修复后 |
| cube×cube OR | ✅ 12 faces | ⚠️ 0 faces | Phase 4 修复后 |
| cube×cube SUB | ✅ 9 faces | ⚠️ 0 faces | Phase 4 修复后 |
| cube×cylinder | ✅ 9 faces | ⚠️ 0 faces | Phase 4+5 |
| sphere SSI | n/a | ✅ r=1,z=0 | 完成 |
| BSpline SSI | n/a | ✅ 4.5e-6 | 完成 |
| plane×plane SSI | ✅ 解析 | ✅ 解析 | 完成 |
| plane×curved SSI | ✅ 二分法 | ✅ 二分法 | 完成 |
