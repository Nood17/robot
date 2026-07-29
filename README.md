# **G1 Robot 展厅自动导览系统**

本项目旨在基于宇树科技（Unitree）G1 机器人，开发一套适用于展厅和展馆的自动导览系统。机器人通过 3D 激光 SLAM 技术实现自主建图与导航，并承担语音播报与领航任务。

## **主要应用场景与功能**

* **核心场景**：展厅 / 展馆自动导览。  
* **G1 机器人角色**：语音播报、导航领航。  
* **建图与导航**：基于 Livox MID360 激光雷达的 3D 激光 SLAM 建图。  
* **语音讲解与交互**：研发中（暂定）。  
* **机器人操控**：目前计划使用 Unitree SDK，后期计划开发配套的平板控制软件。  
* **参考资料**：[宇树 VuiClient Service 音频相关文档](https://support.unitree.com/home/zh/G1_developer/VuiClient_Service)

## **环境与硬件要求**

* **操作系统**：Ubuntu 20.04  
* **ROS 版本**：ROS 1 Noetic（请预先自行安装）  
* **核心传感器**：Livox MID360 激光雷达

## **网络与雷达配置**

在运行代码前，需要确保上位机与 Livox MID360 激光雷达处于同一网段。

1. 保持计算机与机器人的网线物理连接。  
2. 进入系统的“网络-有线连接”设置，将 IPv4 改为手动配置：  
   * **IP 地址**：192.168.123.xxx （xxx 为 0-255 之间的整数，建议避开雷达默认 IP 120）  
   * **子网掩码 (Netmask)**：255.255.255.0  
3. 打开终端，测试与雷达的连接：  
   \# 测试是否能连接到 MID360 的默认 IP  
   ping 192.168.123.120

   *注意：如果无法 ping 通，请检查网络设置是否已保存，或尝试更换上述的 xxx 后缀。*  
4. 修改项目中的雷达 IP 配置文件：  
   \# 进入雷达配置文件目录  
   cd ros\_workspace/src/WK/G1Nav2D/src/livox\_ros\_driver2/config  
   \# 编辑 MID360\_config.json  
   vim MID360\_config.json

   **修改说明**：将其中的 host\_net\_info 节点下的 4 个 IP 地址修改为您刚刚在有线连接中设置的本机 IP（即 192.168.123.xxx）。

## **安装与编译**

### **1\. 下载工作空间**

\# 克隆项目源码  
git clone https://github.com/nood/robot.git

\# 进入工作空间目录  
cd robot/ros\_workspace

### **2\. 安装基础编译工具及依赖**

\# 更新软件源并安装 cmake 及编译基础套件  
sudo apt update   
sudo apt install \-y cmake build-essential git

\# 安装 ROS Noetic 相关的传感器及导航依赖包  
sudo apt-get install \-y ros-noetic-tf2-sensor-msgs \\  
                        ros-noetic-teb-local-planner \\  
                        ros-noetic-global-planner \\  
                        ros-noetic-costmap-server

### **3\. 编译安装 Livox-SDK2**

\# 进入存放 SDK 源码的 src 目录 (假设 SDK 已包含在 src 中)  
cd src  
mkdir build && cd build

\# 编译并安装 SDK 到系统  
cmake ..   
make \-j  
sudo make install

### **4\. 编译 ROS 工作空间**

\# 退回到工作空间根目录  
cd ../.. 

\# 清理旧的编译残留，确保全新编译  
rm \-rf build/ devel/

\# 加载 ROS 基础环境变量  
source /opt/ros/noetic/setup.bash

\# 指定 ROS1 版本，单独编译雷达驱动包  
catkin\_make \-DROS\_EDITION=ROS1 \--pkg livox\_ros\_driver2

\# 单线程编译 fastlio (防止多线程内存不足导致编译失败)  
catkin\_make \-DROS\_EDITION=ROS1 \--pkg fastlio \-j1

\# 编译工作空间内剩余的所有功能包  
catkin\_make \-DROS\_EDITION=ROS1

\# 刷新当前工作空间的环境变量  
source devel/setup.bash

### **5\. 编译与配置宇树运动接口 (CycloneDDS)**

\# 进入 cyclonedds 源码目录  
cd src/WK/cyclonedds

\# 创建编译与安装目录  
mkdir build install && cd build

\# 配置 cmake，指定安装路径为刚刚创建的 install 文件夹  
cmake .. \-DCMAKE\_INSTALL\_PREFIX=../install  
\# 编译并安装  
cmake \--build . \--target install

\# 将路径导出为环境变量（注意：请将 /path/to 替换为您系统的实际绝对路径）  
export CYCLONEDDS\_HOME="/path/to/ros\_workspace/src/WK/cyclonedds/install"

\# 安装 Python 接口包  
\# 排错提示：如果执行失败，可能是由于 pip 版本或环境限制问题，可尝试更新 pip 或使用虚拟环境  
pip3 install \-e . 

为了避免每次打开终端都需要重新配置 CycloneDDS 的环境变量，建议将其追加到 \~/.bashrc 中：

\# 永久追加环境变量到 bashrc（请务必将 /path/to 替换为实际绝对路径）  
echo 'export CYCLONEDDS\_HOME="/path/to/ros\_workspace/src/WK/cyclonedds/install"' \>\> \~/.bashrc

\# 使环境变量立即生效  
source \~/.bashrc

## **使用说明**

### **1\. 3D SLAM 建图**

打开终端，运行 Fast-LIO 建图节点：

\# 进入工作空间并加载环境变量  
cd ros\_workspace  
source devel/setup.bash

\# 启动建图 launch 文件  
roslaunch fastlio mapping.launch

*运行成功后，终端会有持续的数据刷新提示，表示正在接收雷达数据并进行建图。*

### **2\. 保存地图**

当地图扫描完整后，**新开一个终端**，执行以下命令保存地图：

cd ros\_workspace  
source devel/setup.bash

\# 订阅 projected\_map 话题并保存为地图文件  
\# 请将 \[your\_username\] 替换为您的实际系统用户名  
rosrun map\_server map\_saver map:=/projected\_map \-f /home/\[your\_username\]/map/mymap

### **3\. 编辑与修饰地图**

Fast-LIO 投影出来的 2D 地图可能会有一些噪点，可以通过图像编辑软件（如 PhotoGIMP）进行修饰。

1. **修改 YAML 配置文件**：  
   在地图保存路径找到 mymap.yaml 并使用文本编辑器打开：  
   * 将 image: 后面的路径修改为 mymap.pgm 的正确相对或绝对路径。  
   * 将 origin: 后面的 \-nan 修改为 0 （避免 ROS 无法解析原点坐标）。  
2. **编辑 PGM 图像文件**：  
   * 下载并安装 [PhotoGIMP](https://github.com/Diolinux/PhotoGIMP)（或使用原版 GIMP）。  
   * 用软件打开 mymap.pgm，擦除不需要的噪点或补齐墙体轮廓，保存即可供导航模块使用。

## **待办事项 (TODO)**

* \[ \] 接入并调试语音交互模块。  
* \[ \] 完善基于 Unitree SDK 的运动控制逻辑。  
* \[ \] 开发跨平台的平板端控制软件，优化用户交互体验。  
* \[ \] 后续导航参数调优与更新。
