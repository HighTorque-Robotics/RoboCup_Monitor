# monitor
Monitor for displaying robot and competition status.

## Build

```bash
sudo apt-get update

sudo apt-get install \
    libsfml-dev \
    libopencv-dev \
    libjsoncpp-dev \
    libfmt-dev \
    libboost-filesystem-dev \
    libboost-system-dev \
    libtclap-dev
````

本项目使用标准的 CMake 流程:

```bash
# 1. 克隆仓库
# git clone ...
cd dancer-monitor

# 2. 创建一个 build 目录
mkdir build
cd build

# 3. 运行 CMake 来配置项目
cmake ..

# 4. 运行 make 来编译
make
```

编译完成后，你会在 build 目录中找到一个名为 `dmonitor_node` 的可执行文件。

## Usage

### 1\. 实时监控 + 自动录制 (推荐)

默认情况下，程序会自动在 `~/monitor-logs/` 下创建一个以当前时间命名的文件夹，保存 JSON 数据。

```bash
./dmonitor_node
```

### 2\. 实时监控 + 录制摄像头画面

如果你插上了 USB 摄像头（设备 ID 为 0），并希望同时保存比赛视频和数据。
生成文件包含：`monitoring.json` (数据) 和 `log_video.avi` (视频)。

```bash
./dmonitor_node -c 0
```

### 3\. 回放历史比赛 (Replay)

指定一个**已经存在**的日志文件夹路径，程序会自动进入回放模式。

```bash
./dmonitor_node -l /your_log_path
```

### 4\. 纯监视 (静默模式)

如果只是调试代码，不想产生垃圾文件，可以使用静默模式。

```bash
./dmonitor_node -q
```

## Shortcuts

程序运行时的键盘控制说明：

| 按键 | 功能 | 说明 |
| :--- | :--- | :--- |
| **Tab** | **切换进攻视角** | 镜像翻转场地，统一观察进攻方向 (Normal / Inverted) |
| **Esc** | **退出程序** | 关闭窗口并保存日志 |
| **Space** | **暂停/继续** | 仅限回放模式 |
| **F** | **快进 (Fast)** | 仅限回放模式  |
| **S** | **超快进 (Super)** | 仅限回放模式  |
| **B** | **倒带 (Back)** | 仅限回放模式 |


## Test
src/test_monitor.py是一个测试脚本，运行后会广播一个假的机器人消息（ID1,绕着中圈转），用来测试monitor的功能是否正常