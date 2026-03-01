# CADability 布尔运算退化情况完整文档

本文档对 `BRepOperation`（`BRepIntersection.cs`）及其底层几何求交子系统中处理的所有退化/边界情况进行分类总结。

---

## 目录

1. [A 类：切向相交](#a-类切向相交)
2. [B 类：重叠面与对向面](#b-类重叠面与对向面)
3. [C 类：交线-原始边重合](#c-类交线-原始边重合)
4. [D 类：非流形拓扑](#d-类非流形拓扑)
5. [E 类：极点与接缝](#e-类极点与接缝)
6. [F 类：包含关系（无交线）](#f-类包含关系无交线)
7. [G 类：闭合环查找歧义](#g-类闭合环查找歧义)
8. [H 类：曲面-曲面交线退化](#h-类曲面-曲面交线退化)
9. [I 类：数值精度与修补](#i-类数值精度与修补)

---

## A 类：切向相交

Edge 与 Face 几乎平行时，穿刺检测精度急剧下降。

### A1. 切向 Edge-Face 穿刺

- **文件**: `BRepIntersection.cs:1525-1531`
- **几何**: Edge 的方向与 Face 的法线几乎垂直（点积 < 0.01）
- **现象**: 交点位置误差可达正常精度的 1000 倍
- **处理**: 容差从 `Precision.eps` 放大到 `Precision.eps * 1000`

### A2. 切向圆角（Fillet）交线

- **文件**: `BRepIntersection.cs:1535-1565`
- **几何**: 圆角面与相邻面的切向连接处
- **现象**: 已知的交线 Edge 端点与新计算的交点偏差过大
- **处理**: 将新交点吸附到已知交线端点（`prec * 100` 容差）

### A3. 切向 Edge-Face 穿刺在端点

- **文件**: `BRepIntersection.cs:3105-3107`
- **几何**: Edge 在起点或终点处与 Face 切向相交
- **现象**: 在 Edge 参数的 0 或 1 附近产生虚假交点
- **处理**: 当 `|curveDir · surfaceDir| < 1e-3` 且参数接近端点时跳过

### A4. 切向面法线方向判断

- **文件**: `BRepIntersection.cs:9365-9430`
- **几何**: 两个面在交线附近近乎平行
- **现象**: 法线点积接近 0，无法判断交线段是否在 Face 内部
- **处理**: 从 Edge 的 2D 投影向面内部偏移一步采样法线

---

## B 类：重叠面与对向面

两个 Shell 的 Face 共享相同的底层曲面。

### B1. 同向重叠面

- **文件**: `BRepIntersection.cs:3260-3330`
- **几何**: 两个 Face 的曲面几何相同且法线同向
- **现象**: Edge 躺在 Face 上而非穿过，Edge-Face 求交完全失效
- **处理**: `findOverlappingFaces()` 检测 + `CollectOverlappingCommonFaces()` 提取公共部分

### B2. 对向重叠面（抵消面）

- **文件**: `BRepIntersection.cs:3319-3340, 1428-1430`
- **几何**: 两个 Face 几何相同但法线反向（如两个体的粘合面）
- **现象**: 两个面互相抵消
- **处理**: 添加到 `oppositeFaces` / `cancelledfaces`，在结果中移除

### B3. 仅有抵消面无交线

- **文件**: `BRepIntersection.cs:4373-4383`
- **几何**: 两个 Shell 仅通过共享面粘合，无其他交线
- **现象**: 交线集为空但不应返回空结果
- **处理**: union 时移除抵消面后合并剩余面

---

## C 类：交线-原始边重合

交线恰好与原始 Shell 的 Edge 几何重合。

### C1. 同向重复交线

- **文件**: `BRepIntersection.cs:3849-3857`
- **几何**: 同一穿刺产生了两条方向相同的交线 Edge（如 shell2 的 Edge 完全位于 shell1 的 Face 内）
- **现象**: 重复的交线
- **处理**: 丢弃一条，保留一条

### C2. 反向重复交线

- **文件**: `BRepIntersection.cs:3859-3871`
- **几何**: 交线 Edge 与其反向副本同时存在
- **现象**: 拓扑歧义
- **处理**: 两条都删除（互相抵消）

### C3. 交线与原始轮廓 Edge 同向重合

- **文件**: `BRepIntersection.cs:3881-3886`
- **几何**: 交线恰好与 Face 的边界 Edge 重合且方向相同
- **现象**: 多余的 Edge
- **处理**: 移除交线 Edge，保留原始 Edge

### C4. 交线与原始轮廓 Edge 反向重合

- **文件**: `BRepIntersection.cs:3888-3893`
- **几何**: 交线与 Face 的边界 Edge 重合但方向相反
- **现象**: 两条 Edge 互相抵消，该段边界消失
- **处理**: 两条都移除

### C5. 交线与 CommonFace 的 Edge 重合

- **文件**: `BRepIntersection.cs:3967-3975`
- **几何**: 重叠面产生的公共面的边界与交线重合
- **现象**: 重复 Edge
- **处理**: 移除重合的交线 Edge

---

## D 类：非流形拓扑

超过两个 Face 共享同一条 Edge。

### D1. 非流形 Edge（三面共边）

- **文件**: `BRepIntersection.cs:3810-3811, 3868-3871`
- **几何**: 反向重复交线同时与原始 Edge 重合
- **现象**: 一条 Edge 连接了 3 个或更多 Face
- **处理**: 加入 `nonManifoldEdges` 集合，在组装时不参与连接

### D2. 非流形候选面

- **文件**: `BRepIntersection.cs:3998-4001, 4476-4490`
- **几何**: 包含非流形 Edge 的 Face
- **现象**: 可能形成不封闭的 Shell
- **处理**: 标记为 `nonManifoldCandidates`，尝试累积后封闭

---

## E 类：极点与接缝

曲面参数域的退化（如球的极点、圆柱的接缝线）。

### E1. 极点 Edge（Curve3D == null）

- **文件**: `BRepIntersection.cs:1594-1595, 4773-4785`
- **几何**: 球面/锥面的极点处 Edge 无 3D 曲线
- **现象**: 拆分、遍历逻辑无法处理 null 曲线
- **处理**: 跳过 `Curve3D == null` 的 Edge

### E2. 周期面接缝

- **文件**: `BRepIntersection.cs:2062-2063`
- **几何**: 圆柱、环面等曲面的 u/v 具有周期性
- **现象**: 交线可能跨越接缝，2D 坐标出现跳跃
- **处理**: 预处理时调用 `SplitPeriodicFaces()` 沿接缝拆分

---

## F 类：包含关系（无交线）

两个 Shell 完全不相交或一个完全包含另一个。

### F1. Union 不相交

- **文件**: `BRepIntersection.cs:4419-4429`
- **几何**: 两个 Shell 空间完全分离
- **现象**: 无交线，无裁剪
- **处理**: 返回两个 Shell

### F2. Union 一个包含另一个

- **文件**: `BRepIntersection.cs:4423-4424`
- **几何**: Shell2 完全包含 Shell1
- **现象**: 结果应为外层 Shell
- **处理**: 用 `shell.Contains(point)` 判断包含关系

### F3. Difference 被完全包含

- **文件**: `BRepIntersection.cs:4433-4434`
- **几何**: Shell2 在 Shell1 外部（不相交）
- **现象**: Shell1 不受影响
- **处理**: 返回 Shell1

### F4. Difference 产生空腔

- **文件**: `BRepIntersection.cs:4435-4440`
- **几何**: Shell2 完全在 Shell1 内部
- **现象**: 结果应为带空腔的实体（尚未完全实现）
- **处理**: 返回两个 Shell

### F5. Intersection 不相交

- **文件**: `BRepIntersection.cs:4442-4447`
- **几何**: 两个 Shell 空间完全分离
- **现象**: 无公共部分
- **处理**: 返回空数组

### F6. Intersection 一个包含另一个

- **文件**: `BRepIntersection.cs:4444-4445`
- **几何**: Shell2 完全包含 Shell1
- **现象**: 结果应为内层 Shell
- **处理**: 返回被包含的 Shell

---

## G 类：闭合环查找歧义

在 Face 的 2D 参数空间中查找闭合环时的歧义。

### G1. 多条交线从同一顶点出发

- **文件**: `BRepIntersection.cs:4787-4827`
- **几何**: 两个圆柱的切触点，多条交线在一点汇聚
- **现象**: 下一步走哪条 Edge 不确定
- **处理**: "最左转"策略——选择 2D 方向左转角最大的 Edge

### G2. 无外轮廓环（全是孔）

- **文件**: `BRepIntersection.cs:4041-4055`
- **几何**: 交线仅在 Face 内部形成了孔洞（面积为负的环）
- **现象**: 没有外轮廓，但 Face 的原始边界未被触碰
- **处理**: 将 Face 的原始外轮廓补充为外环

### G3. 闭合交线上的点对配对歧义

- **文件**: `BRepIntersection.cs:9231-9250`
- **几何**: 两个曲面的交线是闭合曲线（如两个球交线为圆）
- **现象**: 无法确定哪些穿刺点对之间的曲线段是有效的
- **处理**: 用起点是否在两个 Face 的 Area 内判断；可能需要旋转排序

### G4. 退化的薄壳（两个反向面）

- **文件**: `BRepIntersection.cs:4487, 4502`
- **几何**: 裁剪后产生两个面积相同但方向相反的面
- **现象**: 体积为 0 或极小的 Shell
- **处理**: 用 `Volume() > Precision.eps * 100` 过滤

---

## H 类：曲面-曲面交线退化

底层曲面-曲面交线计算中的退化情况。

### H1. 相切曲面（法线平行）

- **文件**: `Surface.cs:12673-12776`
- **几何**: 两个曲面在交线附近法线近乎平行
- **现象**: 交叉乘积接近零向量，无法确定交线方向
- **处理**: 设置 `dir1 = dir2 = NullVector`，加入 `tangential` 列表特殊处理

### H2. 交点落在参数网格边界上

- **文件**: `Surface.cs:12984-13038`
- **几何**: 交点恰好位于 FixedU/FixedV 参数网格线上
- **现象**: 该交点可能被两个相邻格子各计算一次，或被漏掉
- **处理**: 将该参数偏移 `4 * prec`，然后重新从头计算（`splitted = true`）

### H3. 重新细分超过限制

- **文件**: `Surface.cs:12936-12937`
- **几何**: 反复出现交点落在网格边界上的情况
- **现象**: 细分次数超过 10 次
- **处理**: 放弃，返回已找到的部分交线

### H4. 完全相同的曲面

- **文件**: `Surface.cs:5330-5332`
- **几何**: 两个曲面几何完全相同
- **现象**: 整个曲面都是"交线"
- **处理**: `SameGeometry` 检测后返回空数组

### H5. 无边穿刺的闭合交线环

- **文件**: `Surface.cs:12919-12930, BRepIntersection.cs:2976-3030`
- **几何**: 两个球只在内部相交（交圆不穿过任何 Edge）
- **现象**: Edge-Face 求交找不到任何穿刺点
- **处理**: `createInnerFaceIntersections()` + `additionalSearchPositions`

### H6. InterpolatedDualSurfaceCurve 方向退化

- **文件**: `InterpolatedDualSurfaceCurve.cs:221-251`
- **几何**: 两个曲面在交线上某点法线平行（切触点）
- **现象**: 叉积为零，无法确定切线方向
- **处理**: 回退到近似多项式的切线方向

---

## I 类：数值精度与修补

浮点误差导致的不封闭 Shell 以及各种修补策略。

### I1. 开放边检测与修复

- **文件**: `BRepIntersection.cs:2083-2084, 4465-4471`
- **几何**: 浮点误差导致 Edge 没有正确连接到第二个 Face
- **处理**: `TryConnectOpenEdges()` 尝试几何匹配连接

### I2. 缺失面修补

- **文件**: `BRepIntersection.cs:4493-4507`
- **几何**: 裁剪后 Shell 有开放边，缺少某些面
- **处理**: `TryFixMissingFaces()` 从原始 Shell 中寻找能填补空缺的面

### I3. 八叉树顶点落在格子边界

- **文件**: `BRepIntersection.cs:2169-2175`
- **几何**: 顶点恰好位于八叉树细分平面上
- **现象**: 可能被错误地分配到相邻格子
- **处理**: 将八叉树包围盒偏移 `extsize * 1e-4`

### I4. 几何等价但拓扑独立的 Edge

- **文件**: `BRepIntersection.cs:4355-4367`
- **几何**: 两条 Edge 几何相同但是不同对象
- **现象**: 不通过共享指针连接
- **处理**: `SameEdge` 几何比较 + 手动连接

---

## 精度常量汇总

| 位置 | 表达式 | 用途 |
|------|--------|------|
| `BRepIntersection.cs:1530` | `Precision.eps * 1000` | 切向穿刺点距离容差 |
| `BRepIntersection.cs:1546` | `prec * 100` | 吸附到已知交线端点 |
| `BRepIntersection.cs:2175` | `extsize * 1e-6` | 八叉树精度 |
| `BRepIntersection.cs:3107` | `Precision.eps * 100` | 端点切向穿刺跳过 |
| `BRepIntersection.cs:4487` | `Precision.eps * 100` | 最小体积过滤 |
| `BRepIntersection.cs:9159` | `100 * Precision.eps` | 已知切向交线吸附 |
| `Surface.cs:12937` | `splitcount > 10` | 参数网格细分上限 |
