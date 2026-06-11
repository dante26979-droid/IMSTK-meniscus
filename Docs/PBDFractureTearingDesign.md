# PBDFracture 撕裂断面可视化方案记录

## 背景

`Examples/PBD/PBDFracture` 当前用于展示基于 iMSTK PBD/XPBD 的渐进式撕裂效果。物理层仍采用轻量方案：沿预设撕裂路线逐步 deactivate tet-edge distance constraints，并通过 code-driven pull 将上层一侧抬起。当前版本不做真实 tet 拓扑 split，也不新增 PBD 物理粒子。

断面可视化使用 iMSTK 原生 `SurfaceMesh` / `VisualModel`。目标是先得到一个稳定、可解释、可扩展的 demo，再逐步靠近 PBD-Interval 中 patch/anchor 风格的断裂显示重建。

## 当前方案：断面贴合对应 tet 层

当前采用的方案是 **side-layer snap**：

1. 在 rest configuration 中用撕裂面切 tet。
2. 对每个被切 tet，计算切面与 tet 边的交点。
3. 对上、下两个断面分别选择对应 side 的 tet 顶点作为锚点。
4. 更新断面顶点时，去掉交点相对锚点的法向 offset，只保留切面切向 offset。
5. 因此水平分层撕裂时：
   - 上断面贴合上层 tetmodel 网格。
   - 下断面贴合下层 tetmodel 网格。
   - 不再把断面悬在两个离散 tet 层之间。

这样解决了之前看到的视觉问题：切面位于两层 tet 顶点中间时，上下两侧都会少一层视觉厚度，导致明显 gap。贴层后，断面和当前离散 tet 网格一致，demo 的视觉结果更稳定。

### 当前方案的优点

- 不修改 PBD body 顶点数组、tet cells 或约束拓扑。
- 不需要引入 PBD-Interval 的可视化 mesh sidecar。
- 对当前规则 block 和水平分层撕裂足够稳定。
- 对未来不规则切面仍有一定泛化性，因为 snap 是沿 cut normal 处理，而不是硬编码某个世界轴。

### 当前方案的限制

- 断面不是严格几何切面，而是被吸附到对应 side 的离散 tet 层。
- 如果 tet 分辨率较低，断面会表现出明显的体网格层级感。
- 它是展示性方案，不表示真实材料拓扑被 split。
- 对复杂外表面，cap surface 和 boundary clipped surface 仍然是两套可视化几何，长期看最好合并为统一 patch builder。

## 可选优化 1：补充 bridge / skirt mesh

另一种可选路径是保留原本的切面生成方式，也就是让断面仍位于真实切割几何位置，然后补充连接面片：

1. cap mesh 仍按真实切面生成。
2. boundary mesh 仍按原 tetmodel 外表面裁剪。
3. 在 cap 边界和对应 side 的 tetmodel 外表面边界之间补一圈 bridge/skirt triangles。
4. 上断面和下断面各自生成自己的 bridge mesh。
5. bridge mesh 只参与可视化，不新增 PBD 顶点，不改变 tet cells，不改变 constraints。

这个方式可以理解为：保持物理和切割几何不变，用额外的显示 patch 把新断面和原模型表面缝起来。

### 适用场景

- 希望断面更接近真实切割几何位置，而不是吸附到 tet 层。
- 模型分辨率较低，但又希望视觉上看不到层间空洞。
- 后续要接近 PBD-Interval 的 fracture-display patch 模型。

### 实现要点

- 建议新增 demo-local helper，例如 `PbdFractureBridgeMeshBuilder`。
- 输入包括：
  - cap mesh 边界 loop 或 cut-point records。
  - clipped boundary mesh 的对应边界 loop。
  - side sign。
  - 当前 particle positions。
  - rest positions。
- 输出一个或两个 `SurfaceMesh`：
  - lifted side bridge mesh。
  - stationary side bridge mesh。
- bridge 顶点应使用与 cap/boundary 一致的 anchor 更新规则，避免每帧漂移。

### 风险

- 需要稳定配对 cap boundary 和 clipped boundary。如果 loop 不连续或切面不规则，简单按最近点配对可能会产生扭曲三角形。
- 多段撕裂、分叉撕裂、非流形边界下，需要 loop extraction 和拓扑排序。
- 若 bridge 与 cap/boundary 仍然分属不同 mesh，透明材质下可能有排序伪影；更好的方式是最终合并为每侧一个 patch mesh。

## 可选优化 2：每侧统一 patch mesh

比 bridge mesh 更完整的方案是为每个断裂 side 构建一个统一 visual patch：

1. side exterior clipped faces。
2. side fracture cap faces。
3. side bridge/skirt faces。
4. 共享同一套 patch vertices / anchors。

这样可避免 cap、bridge、外表面之间因为独立重建产生微小裂缝。这个方向更接近 PBD-Interval 的 `FracturePatchMesh` 思路。

### 推荐数据模型

可引入轻量的 demo-local patch vertex anchor：

```cpp
struct FractureVisualAnchor
{
    std::array<int, 4> particleIds;
    Vec4d weights;
    Vec3d offset;
    int sideSign;
};
```

更新时：

1. 用 `particleIds + weights` 计算当前 anchor position。
2. 加上 `offset` 得到 render vertex position。
3. 所有 cap、bridge、clipped boundary face 共用同一套 anchor 顶点。

这个方案不要求修改 PBD 物理顶点结构，但能让可视化断面随着模型运动稳定贴合。

## 可选优化 3：真实拓扑 split

真正物理意义上的撕裂需要新增或复制物理顶点，并更新 tet 拓扑和约束拓扑：

1. 沿断裂面复制 shared vertices。
2. 将 tet cells 按 component 重新绑定到不同 vertex copy。
3. 重建或局部更新 constraints。
4. 更新 collision/visual mapping。
5. 维护 component、fracture state 和 haptic interaction state。

这条路径会影响 PBD body 的顶点数据结构和约束生命周期，复杂度明显高于当前 demo。建议在展示性断裂稳定后再单独设计，不应混入当前 `PBDFracture` 的第一版 demo。

## 当前建议

短期保留当前 side-layer snap 方案，作为稳定可运行 demo：

- 展示右上角上提。
- 沿预设路线渐进撕裂。
- 左侧保留未完全断开的连接区域。
- 上下断面贴合对应 tet 层。
- 不修改物理拓扑。

中期可以新增 bridge/skirt mesh 作为可选显示模式：

- 保留真实切面 cap。
- 额外补齐 cap 与 tetmodel 外表面之间的连接面。
- 仍然只改 visual mesh。

长期若要支持不规则模型、不规则路径、haptic 实时撕裂和高质量断面，可收敛到统一 patch mesh + barycentric anchor 的实现方向。
