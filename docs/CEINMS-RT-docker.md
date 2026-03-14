# 在 Docker 中复现 CEINMS-RT 记录

**项目地址**：https://github.com/CEINMS-RT/ceinmsrt-core-cpp  
**项目论文**：Sartori et al., 2025, "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots" (IEEE Transactions on Medical Robotics and Bionics)  
**复现目标**：在本地复现实时 EMG-driven 神经肌肉骨骼模型，跑通实时/离线模式，生成关节力矩、肌肉力等输出。

## 设备信息

- **笔记本**：联想拯救者 Y7000P 2022  
  - CPU：Intel Core i7-12700H (14 核 20 线程)  
  - 内存：16GB DDR4  
  - 显卡：NVIDIA RTX 3050Ti Laptop GPU 4GB  
- **操作系统**：Ubuntu 22.04 LTS (Jammy Jellyfish)  
  - 桌面环境：GNOME on Xorg
- **Docker**：Docker Desktop for Linux  

## 复现过程 & 踩坑总结

### 准备阶段

> 最开始看到该项目中涉及的包非常多，其中大部分我之前已经装在windows上了，我担心直接跑要解决很多版本冲突的bug污染环境，于是选择到笔记本上的Ubuntu22.04系统上运行该项目。

先Clone项目：

```bash
git clone https://github.com/CEINMS-RT/ceinmsrt-core-cpp
```

拉取官方镜像（该docker image中已预装 OpenSim）:

```bash
docker pull be1et/ceinms-platform:latest
```

> docker desktop不能直连，我尝试了很多国内镜像源均失效。最后发现可以在docker desktop的`settings`-`Resources`中将`Proxies-Proxy mode`改为`Manual configuration`，根据clash上的端口填写就可以直接访问了，我将Bypass proxy设置为：
>
> ```
> localhost,127.0.0.1,::1,*.docker.internal,hubproxy.docker.internal,192.168.0.0/16,172.17.0.0/16,172.18.0.0/16,10.0.0.0/8
> ```

### 启动容器
拉取好镜像之后启动容器，经过反复尝试我总结出以下命令：
```bash
docker run -it --rm \
  --name ceinms-rt \
  -e DISPLAY=:1 \
  -e QT_X11_NO_MITSHM=1 \
  -e QT_QPA_PLATFORM=xcb \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority:ro \
  -v $(pwd):/workspace/ceinmsrt-core-cpp \
  --net=host --privileged --ipc=host \
  be1et/ceinms-platform:latest /bin/bash
```

我的Linux桌面环境让xeyes 不能正常显示，我先禁用 GUI。

现在就已经进入容器了

> 也可以在docker中直接run

### 编译核心
```bash
cd /workspace/ceinmsrt-core-cpp
rm -rf build

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DOpenSim_DIR=/opensim_install/lib/cmake/OpenSim

cmake --build build -j$(nproc)  
```

期间报错：`c++: fatal error: Killed signal terminated program cc1plus` → Docker Desktop 内存默认低，Settings → Resources → Memory 增加为 8GB即可。

最终编译成功，生成 `bin/Unix/CEINMS` 和 `calibrate`

### 运行测试 
先 clone 示例模型：
```bash
cd /workspace/ceinmsrt-core-cpp/bin/Unix
git clone https://github.com/CEINMS-RT/LowerLimbModel.git
```

**注意：**

- 在`workspace/ceinmsrt-core-cpp/bin/Unix`下新建`cfg`文件夹，把`LowerLimbModel`中的`SplineCoeff`复制到`cfg`中，便于`ceinms`读取。

  > 最好将`LowerLimbModel`文件夹克隆到cfg文件夹，便于用相对路径

- 这个下肢模型是基于win版本开发的，其没有做操作系统识别，所以在`executionRT.xml`文件中需要将`Device`改为适用于linux的`.so`文件（原文件中是适用于win的`.dll`）
  ```xml
  <EMGDevice>libPluginEMG0.so</EMGDevice>
  	<EMGDeviceFile>cfg/LowerLimbModel/executionEMG.xml</EMGDeviceFile>
  	<!-- <AngleDevice>plugin/RTOSIM_Plugin/lib/Debug/RTOSIMPlugin.dll</AngleDevice> -->
  	<AngleDevice>libPluginAngle0.so</AngleDevice>
  	<AngleDeviceFile>cfg/LowerLimbModel/executionIK_scale.xml</AngleDeviceFile>
  ...
  ```

**成功运行命令**：

```bash
cd /workspace/ceinmsrt-core-cpp/bin/Unix
# 先在命令行内输出，不启用GUI
export QT_QPA_PLATFORM=offscreen
  
./CEINMS \
  -e /workspace/LowerLimbModel/executionRT.xml \
  -s /workspace/LowerLimbModel/data/subjectCalibrated.xml \
  -r results_test_walk36 \
  -v 3
```

日志显示开始计算：

```bash
QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'

+-+-+-+-+-+-+-+-+-+
|C|E|I|N|M|S|-|R|T|
+-+-+-+-+-+-+-+-+-+-+
|C|a|l|i|b|r|a|t|e|d|
+-+-+-+-+-+-+-+-+-+-+-+-+
|E|M|G|-|I|n|f|o|r|m|e|d|
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|N|e|u|r|o|m|u|s|c|u|l|o|s|k|e|l|e|t|a|l|
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|R|e|a|l|-|t|i|m|e|
+-+-+-+-+-+-+-+-+-+
|T|o|o|l|b|o|x|
+-+-+-+-+-+-+-+
|L|I|N|U|X|
+-+-+-+-+-+-+-+

Copyright (C) 2026
David LLoyd, Monica Reggiani, Massimo Sartori, Claudio Pizzolato, Guillaume Durandau

/workspace/ceinmsrt-core-cpp/src/CEINMS.cpp: 256 -- Model: RealTimeOpenLoopExponentialActivationStiffTendonOnline.
libPluginAngle0.so
/workspace/ceinmsrt-core-cpp/lib/Producers/DevicePlugin/EMG0plugin.cpp: 26 -- init EMG lib
Plugin Angle0plugin, initialisation done.
/workspace/ceinmsrt-core-cpp/lib/Producers/DevicePlugin/EMG0plugin.cpp: 49 -- Plugin EMG0plugin, initialisation done.
/workspace/ceinmsrt-core-cpp/lib/Producers/FromExtDevice/LmtMaFromMTUSpline.cpp: 79 -- LMT: waiting for dof names...
/workspace/ceinmsrt-core-cpp/lib/Producers/FromExtDevice/LmtMaFromMTUSpline.cpp: 89 -- LMT: waiting for muscle names...
/workspace/ceinmsrt-core-cpp/lib/Producers/FromExtDevice/LmtMaFromMTUSpline.cpp: 98 -- LMT: waiting for muscle names... Done
/workspace/ceinmsrt-core-cpp/lib/Producers/FromExtDevice/LmtMaFromMTUSpline.cpp: 125 -- LMT: waiting for ready to start...
/workspace/ceinmsrt-core-cpp/lib/ModelEvaluation/ModelEvaluationRealTime.cpp: 219 -- NMS: waiting for ready To Start ...
...
```

在输出文件夹 `results_test/`中会生成以下文件：

- Activations.sto
- MuscleForces.sto
- Torque.sto（关节力矩，最重要）
- FibreLengths.sto / FibreVelocities.sto
- lmt.sto / ma_*.sto（力臂）
- emg.sto 等

不过由于当前运行没有外部数据输入，所以以上文件中均为零输入下的计算结果（恒定值），只能证明当前项目配置能跑通。

接下来加入`LowerLimbModel`文件夹中的`walk36`数据试试：

```bash
./CEINMS \
  -e /workspace/LowerLimbModel/executionRT.xml \
  -s /workspace/LowerLimbModel/data/subjectCalibrated.xml \
  -p /workspace/LowerLimbModel/data/walk36 \
  -r results_test_walk36 \
  -v 3
```

输出报错：

```bash
QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'

+-+-+-+-+-+-+-+-+-+
|C|E|I|N|M|S|-|R|T|
+-+-+-+-+-+-+-+-+-+-+
|C|a|l|i|b|r|a|t|e|d|
+-+-+-+-+-+-+-+-+-+-+-+-+
|E|M|G|-|I|n|f|o|r|m|e|d|
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|N|e|u|r|o|m|u|s|c|u|l|o|s|k|e|l|e|t|a|l|
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|R|e|a|l|-|t|i|m|e|
+-+-+-+-+-+-+-+-+-+
|T|o|o|l|b|o|x|
+-+-+-+-+-+-+-+
|L|I|N|U|X|
+-+-+-+-+-+-+-+

Copyright (C) 2026
David LLoyd, Monica Reggiani, Massimo Sartori, Claudio Pizzolato, Guillaume Durandau

/workspace/ceinmsrt-core-cpp/src/CEINMS.cpp: 256 -- Model: RealTimeOpenLoopExponentialActivationStiffTendonOnline.

/workspace/ceinmsrt-core-cpp/lib/Producers/FromExtDevice/DynLib.cpp: 48 -- Cannot open plugin because the file `/workspace/ceinmsrt-core-cpp/bin/Unix/.so` cannot be opened - verify the path is correct and the file exists
Segmentation fault
```

`/workspace/ceinmsrt-core-cpp/bin/Unix/.so` cannot be opened 这个报错我暂时还没有解决，我现在排除到的bug来源是：

- 当加上` -p `参数时，CEINMS 内部会触发 **“offline mode override”**（论文里提到 -p 会 overrule execution.xml）。这个 override 逻辑把 <ConsumerPlugin> 里的 <EMGDevice> 和 <AngleDevice> **全部清空或跳过解析**了，导致传给 DynLib 的 libpath 变成空字符串。

但我尝试了禁用插件以及直接在`executionRT.xml`指定数据文件，依然无法离线计算。

> 遂转至windows自己编写python项目复现算法，如前两节所见。
