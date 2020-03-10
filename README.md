# ROS2 cross compile using custom cross-compile toolchain

ROS2交叉编译官方教程都是到ARM平台下的Ubuntu系统, 有完整的包管理器, 所以安装依赖库和Python都比较方便．这里主要记录一下如何使用厂商提供的交叉编译工具链和文件系统,交叉编译`Ros2-base`CPP包到裁剪过的Linux系统, 以`Hi3559CV100`芯片为例. 因为厂商已经提供了最小文件系统, 所以没有使用Docker来制作Arm下的`sysroot`.
## 设置Hisi3559的开发环境

按照SDK里的文档`Hi3559A╱C V100 SDK 安装及升级使用说明`先搭建开发环境

1. 安装依赖包
    ```
    sudo apt-get install make libc6:i386 lib32z1 lib32stdc++6 lib1g-dev libncurses5-dev ncurses-term ibncursesw5-dev g++ u-boot-tools:i386 texinfo texlive gawk libssl-dev openssl bc
    ```

2. 安装交叉编译工具链
    ```
    # 需要root权限
    tar -zxf aarch64-himix100-linux.tgz
    chmod +x aarch64-himix100-linux.install
    ./aarch64-himix100-linux.install
    source /etc/profile
    ```

安装完成之后, 在`/opt/hisi-linux/x86-arm/aarch64-himix100-linux/target`目录下有对应的文件系统，在`/opt/hisi-linux/x86-arm/aarch64-himix100-linux/bin/`目录下有对应的交叉编译工具链

# 获取Ros2源码
这里参照了[ROS2 toolchain cross-compiler](https://github.com/emersonknapp/ros2-toolchain-crosscompile.git)来获取Ros2的最小化Cpp包的源码.

### 添加 Ros2 源
```
curl http://repo.ros2.org/repos.key | apt-key add - && sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
```
### 安装编译必须的工具
```
apt-get update && apt-get install -y \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-lark-parser \
    python3-pip \
    wget
```
### 安装 mixin
```
python3 -m pip install -U colcon-mixin
```
### 安装 vcstool
```
apt install python3-vcstool
```
### 下载源码
```
mkdir src
vcs import src < minimal_cpp_ros2_master.repos
```
### 禁用log4cxx和demos
```
touch src/ros2/rcl_logging/rcl_logging_log4cxx/COLCON_IGNORE
touch src/ros2/demos/COLCON_IGNORE
```

# 交叉编译
`colcon mixin`可以根据需求配置不同的编译参数, 用来交叉编译非常方便.
### 配置colcon mixin
```
colcon mixin add cc file:///cc-mixin-index.yaml
colcon mixin update cc
```
### 编译
交叉编译可执行程序 `component_container` 和 `component_container_mt` 时会遇到动态库链接错误的问题, 所以先添加 `--continue-on-error` 参数, 确保所有的库编译完成.
```
export ROS2_INSTALL_PATH=/ros2_cross_compile/install_hisi
colcon build --mixin aarch64-linux --install-base install_hisi --merge-install --continue-on-error
```
# Troubleshooting
查看编译`log`, 发现是 `libddsc.so.0`, `libyaml.so`, `libPocoFoundation.so.50` 这三个库没有被链接到, 尝试修改`src/ros2/rclcpp/rclcpp_components/CMakeLists.txt` :
```
find_package(yaml REQUIRED)
find_package(Poco REQUIRED Foundation)
find_package(CycloneDDS REQUIRED CONFIG)

target_link_libraries(component_container 
    component_manager
    CycloneDDS::ddsc
)
ament_target_dependencies(component_container
    "rclcpp"
    "yaml"
    "Poco"
)

target_link_libraries(component_container_mt 
    component_manager
    CycloneDDS::ddsc

)
ament_target_dependencies(component_container_mt
    "rclcpp"
    "yaml"
    "Poco"
)
```
最后尝试编译自己的 `Ros Package` 时也要手动链接上面三个库才能编译成功. 

尝试本地编译没有遇到这个问题, 对比分析发现这三个库都是编译时需要先下载再编译的, 可能是交叉编译配置 `CMAKE_SYSROOT` 和 `CMAKE_FIND_ROOT_PATH` 之后导致 `ament_target_dependencies` 不能正确解析包之间的依赖关系, 发生动态库链接错误.