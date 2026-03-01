# 布尔运算测试对比：CADability vs OCCT

## 一、规模对比

| 维度 | CADability | OCCT |
|------|-----------|------|
| **布尔运算测试总数** | 22 | **4,092**（+ 1,094 回归 bug 测试） |
| **测试语言** | C# (MSTest) | Tcl 脚本（DRAW 测试框架） |
| **测试数据** | 代码内构造几何体 | 代码构造 + 外部 `.brep` / `.rle` 文件（~2500 个） |
| **核心代码量** | ~11,000 行（单文件 `BRepIntersection.cs`） | ~50,000+ 行（分布在 12 个 `BOPAlgo_PaveFiller_*.cxx` + Builder 等文件） |

**差距：约 200 倍。** OCCT 的布尔运算测试是经过 20+ 年工业级 CAD 应用积累的结果。

---

## 二、测试网格（Grid）结构对比

### OCCT 的 35 个测试网格

| 网格 | 测试数 | 说明 |
|------|--------|------|
| `bfuse_simple` | 102 | 简单几何体 Fuse（旧 API） |
| `bfuse_complex` | 165 | 复杂工业零件 Fuse |
| `bcut_simple` | 110 | 简单几何体 Cut |
| `bcut_complex` | 153 | 复杂工业零件 Cut |
| `bcommon_simple` | 83 | 简单 Common |
| `bcommon_complex` | 25 | 复杂 Common |
| `bopfuse_simple` | 375 | 简单 Fuse（新 BOP API） |
| `bopfuse_complex` | 128 | 复杂 Fuse（新 BOP API） |
| `bopcut_simple` | 379 | 简单 Cut（新 BOP API） |
| `bopcut_complex` | 140 | 复杂 Cut |
| `bopcommon_simple` | 378 | 简单 Common（新 BOP API） |
| `bopcommon_complex` | 115 | 复杂 Common |
| `boptuc_simple` | 373 | 简单 TUC（tool-until-cut） |
| `boptuc_complex` | 73 | 复杂 TUC |
| `bsection` | 164 | 截面运算（旧 API） |
| `bopsection` | 48 | 截面运算（新 API） |
| `bcutblend` | 1 | 切割+倒圆 |
| `bfuse_2d` / `bcut_2d` / ... | ~545 | **2D 布尔运算**（线框/平面） |
| `volumemaker` | 74 | 从面集构建体积 |
| `cells_test` | 68 | CellsBuilder（任意布尔组合） |
| `splitter` | 12 | 通用分割（Partition） |
| `gdml_private` | 325 | GDML 几何（粒子物理探测器） |
| `gdml_public` | 25 | GDML 公开测试 |
| `history` | 9 | 历史追踪（哪些形状被删除/修改） |
| `removefeatures` | 58 | 特征移除 |
| `opensolid` | 9 | **开放实体**布尔运算 |
| `periodicity` | 6 | **周期性形状** |
| `simplify` | 5 | 结果简化 |
| `mkconnected` | 5 | 连接对象构建 |

### CADability 的测试分类

| 类别 | 测试数 | 说明 |
|------|--------|------|
| A - 切向相交 | 1 | 切向圆柱 |
| B - 重叠/对向面 | 3 | 同面、同位 |
| C - 交线-边重合 | 1 | 边相切 |
| E - 极点与接缝 | 2 | 球面 |
| F - 包含关系 | 6 | 分离/包含 |
| G - 环查找歧义 | 1 | 齐面剪切 |
| H - 曲面交线退化 | 1 | 同面 intersection |
| I - 数值精度 | 1 | 微小间隙 |
| 标准 + 一致性 | 5 | 基本操作 + 体积守恒 |
| 平面分割 | 1 | SplitByPlane |

---

## 三、关键差异分析

### 1. 几何体类型覆盖

| 几何类型组合 | OCCT | CADability |
|------------|------|-----------|
| Box × Box | ✅ 大量（旋转、偏移、嵌套） | ✅ 有 |
| Sphere × Box | ✅ 有 | ✅ 有 |
| Cylinder × Box | ✅ 大量 | ✅ 有 |
| Cylinder × Cylinder | ✅ 有 | ✅ 1个（切向） |
| Sphere × Sphere | ✅ 有 | ✅ 有 |
| Cone × Box | ✅ 有 | ❌ 缺失 |
| Torus × * | ✅ 有 | ❌ 缺失 |
| NURBS × * | ✅ 大量（complex 系列） | ❌ 缺失 |
| 工业零件（STEP/BREP 文件） | ✅ 2500+ 文件 | ❌ 缺失 |
| 2D 布尔（Wire/Edge） | ✅ 545 个测试 | ❌ 缺失 |

**CADability 缺少**: 锥面、环面、NURBS 自由曲面、以及基于真实工业零件的测试。

### 2. 验证方法对比

| 验证手段 | OCCT | CADability |
|---------|------|-----------|
| **表面积校验** (`checkprops -s`) | ✅ 每个测试都有精确值 | ❌ 无 |
| **体积校验** (`checkprops -v`) | ✅ 常用 | ✅ 部分测试有（近似） |
| **拓扑计数** (`checknbshapes -face -edge`) | ✅ 精确拓扑验证 | ❌ 无 |
| **形状合法性** (`checkshape`) | ✅ 自动（end 脚本） | ✅ 开放边检查 |
| **截面无瑕疵** (`checksection`) | ✅ 有 | ❌ 无 |
| **视觉对比** (`checkview`) | ✅ 自动截图 | ❌ 无 |
| **不崩溃** | 隐含 | ✅ 主要验证方式 |
| **体积守恒** (`|A∪B| = |A| + |B| - |A∩B|`) | ❌ 无显式 | ✅ 1个测试 |

**CADability 的主要验证是"不崩溃 + 有结果"，OCCT 要求精确到小数点后的面积/体积/拓扑数。**

### 3. 退化情况覆盖对比

| 退化类别 | OCCT 覆盖 | CADability 覆盖 | 差距 |
|---------|----------|----------------|------|
| **切向面** | ✅ `bug25966` 专用测试 + simple 系列中的旋转 Box | ⚠️ 1 个圆柱切向 | OCCT 有真实 BREP 文件测试 |
| **重叠/共面** | ✅ `history/A3,A5`（coincident 面） | ✅ 3 个测试 | 类似 |
| **包含关系** | ✅ simple 系列中隐含 | ✅ 6 个测试 | CADability 更显式 |
| **极点/接缝** | ✅ sphere/cone 在 simple 中大量出现 + `periodicity` 网格 | ✅ 2 个 | OCCT 更多 |
| **开放实体** | ✅ `opensolid` 9 个测试 | ❌ 无 | OCCT 独有 |
| **自相交** | ✅ `CheckSelfInterference` (PaveFiller_11) | ❌ 无 | OCCT 独有 |
| **Fuzzy（模糊容差）** | ✅ BOPAlgo 支持 fuzzy value | ✅ 1 个微小间隙 | OCCT 更系统化 |
| **非流形** | ✅ `opensolid` 系列 | ⚠️ 测试中无显式非流形 | OCCT 更好 |
| **2D 布尔** | ✅ 545 个 | ❌ 无 | OCCT 独有 |
| **历史追踪** | ✅ `history` 网格 | ❌ 无 | OCCT 独有 |
| **工业回归** | ✅ 1,094 个 bug 回归测试 | ❌ 无 | OCCT 独有 |

### 4. 算法架构差异导致的测试差异

| 特性 | OCCT (BOPAlgo) | CADability (BRepOperation) |
|------|---------------|---------------------------|
| **求交阶段分离** | 11 个编号文件 (PaveFiller_1 到 _11) 各处理一个拓扑级别 | 单文件 11,185 行中 `prepare()` + `Result()` |
| **测试双 API** | 旧 API (`bfuse`) + 新 API (`bopfuse`) 都测试 | 仅一套 API |
| **截面运算** | 独立的 `bsection` + `bopsection`（212 个测试） | 无独立截面测试 |
| **CellsBuilder** | 任意布尔组合（68 个测试） | 无等价功能 |
| **Partition/Split** | 通用分割器（12 个测试） | 仅 `SplitByPlane` |
| **特征移除** | 58 个测试 | 无 |

---

## 四、CADability 测试改进建议

按优先级排序：

### 优先级 1：精确验证（最关键差距）

OCCT 每个测试都验证精确的表面积/体积值。CADability 应该对每个测试添加：
- `Shell.Volume()` 精确值断言
- `Shell.Area()` 精确值断言（如果 API 支持）
- 拓扑计数（face/edge/vertex 数量）

### 优先级 2：增加几何体类型

| 需要增加的组合 | 对应退化类别 |
|--------------|------------|
| Cone × Box | E（极点） |
| Torus × Box | E（接缝+周期面） |
| Cylinder × Cylinder（正交穿过） | H（曲面交线） |
| Sphere × Cylinder | H + E |
| Box × Box（带旋转） | A（切向边-面） |

### 优先级 3：模仿 OCCT 的系统化方法

- **每种操作 × 每种几何组合的矩阵**：OCCT 的 `simple` 系列本质上是 `{box, sphere, cylinder, cone} × {box, sphere, cylinder, cone}` 的交叉测试
- **旋转/平移变换**：同一几何配置在不同空间方位下测试（OCCT 的 `K1-K9` 系列就是同一配置不同旋转角度）
- **渐进式复杂度**：simple → complex → industrial parts

### 优先级 4：CADability 独有但应保留的测试

CADability 有一些 OCCT 没有的测试模式，值得保留：
- **体积守恒公式验证** (`|A∪B| = |A| + |B| - |A∩B|`)——这是一种元验证策略
- **退化情况的显式分类和命名**——便于理解和维护

---

## 五、测试模式对比示例

### OCCT 的典型测试模式

```tcl
# tests/boolean/bfuse_simple/A1
psphere s 1                    # 创建单位球
box b 1 1 1                    # 创建单位立方体
bfuse result s b               # 执行 Fuse
checkprops result -s 14.6394   # 验证表面积
checkview -display result ...  # 视觉截图
```

每个测试：**3-5 行，目标值精确到 4-6 位有效数字**。

### CADability 的典型测试模式

```csharp
[TestMethod]
public void F2_Union_ContainedBox_ReturnsOuter()
{
    var outer = MakeBox(..., 20, 20, 20);
    var inner = MakeBox(..., 10, 10, 10);
    var result = Solid.Unite(outer, inner);
    AssertValidSolid(result, "...");
    double volume = result.Shells[0].Volume(Eps);
    Assert.IsTrue(Math.Abs(volume - 8000.0) < 1.0, "...");
}
```

每个测试：**8-15 行，验证"结果有效" + 粗略体积**。

### 关键差距

1. **OCCT 用精确面积，CADability 用粗略体积** → 面积能发现更多微妙的拓扑错误
2. **OCCT 测试数据来自真实工业案例**（`.brep` 文件）→ 覆盖了代码作者无法预见的几何退化
3. **OCCT 有新旧两套 API 的平行测试** → 确保 API 兼容性

---

## 六、总结

| 方面 | CADability | OCCT | 评价 |
|------|-----------|------|------|
| 测试数量 | 22 | 5,186 | OCCT 是 CADability 的 **236 倍** |
| 退化分类 | 9 类 30+ 子项（文档化） | 隐含在测试中 | CADability 文档更好 |
| 验证精度 | 粗略体积 + 不崩溃 | 精确面积/体积/拓扑数 | OCCT 远超 |
| 几何覆盖 | Box, Sphere, Cylinder | + Cone, Torus, NURBS, 工业零件 | OCCT 远超 |
| 2D 布尔 | 无 | 545 个测试 | OCCT 独有 |
| 开放/非流形 | 无 | 9+ 个测试 | OCCT 独有 |
| 回归测试 | 无 | 1,094 个 | OCCT 独有 |
| 独特优势 | 退化分类文档、体积守恒公式 | 工业级覆盖 | 各有所长 |
