# CEINMS-RT 复现

## 简介与复现目标

基于论文 *《CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots》*

**论文开源资源：**

- **CEINMS-RT 核心的C++源代码：**https://github.com/CEINMS-RT/ceinmsrt-core-cpp
- **CEINMS-RT windows版本的安装脚本：**https://github.com/CEINMS-RT/CEINMS-RT_Installer
- **CEINMS-RT 下肢模型与数据集：**https://github.com/CEINMS-RT/LowerLimbModel
- **CEINMS-RT 官方文档：**https://ceinms-docs.readthedocs.io/en/latest/

**核心复现目标：**

1. **理解并验证前向动力学管线**：实现从表面肌电信号（sEMG）到肌肉激活度（Activation），再到肌肉力（Muscle Force），最后映射为关节力矩（Joint Moment）的完整物理计算链路。
2. **验证其实时性优化策略**：原论文的核心贡献在于打破了离线优化模型的算力瓶颈。本次复现重点实现了其提到的两大提速策略：
   - **运动学查表查值**：离线拟合多维 B 样条曲线（B-splines），将复杂的 OpenSim 肌肉运动学计算转化为 O(1) 复杂度的实时查询。
   - **刚性肌腱假设（Stiff Tendon）**：消除弹性肌腱模型中耗时的隐式微分方程，代之以解析解。
3. **精度评估**：将前向估计的关节力矩与 OpenSim 逆动力学（Inverse Dynamics, ID）的计算结果进行对比分析。
4. **肌肉参数校准**。

## 导航

按照时间顺序我的复现进度如下：

- [🐳 在 Docker 中复现 CEINMS-RT 记录](CEINMS-RT-docker.md) 
  > 尝试在Linux中先用作者提供的docker容器跑通项目，实时计算功能测试成功，但离线计算仍有bug未解决。

- [📊 CEINMS-RT 核心算法复现报告](CEINMS-RT-Core.md) 
  > 在windows上基于Python复现了前向动力学管线，验证了实时性优化策略。力矩计算与OpenSim ID结果的误差在论文范围以内，计算延迟从30ms降至3ms。

- [📊 CEINMS-RT 参数校准算法复现与验证](Calibration.md) 
  > 复现了参数校准算法，成功校准了模型参数，降低了力矩计算误差，但存在可优化空间。