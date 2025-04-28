# 树莓派4B智能车控制系统

## 硬件配置
- 树莓派4B (4GB RAM)
- CSI摄像头 (用于目标检测)
- USB摄像头 (用于循线)
- Arduino Uno (用于车辆控制)

## 软件架构
```
main_controller.py (主控制程序)
├── camera_processor.py (YOLO目标检测)
└── line_follower.py (循线检测)
```

## 功能特性
1. **目标检测**:
   - 使用YOLOv8n预训练模型进行实时检测
   - 支持加载自定义模型(best.pt)
   - 检测到障碍物自动急停

2. **循线功能**:
   - 基于颜色识别的线路跟踪
   - 自动校准线路颜色
   - 支持复杂路径识别

3. **控制逻辑**:
   - 双摄像头并行处理
   - 心跳机制确保连接可靠
   - 完善的异常处理

## 安装依赖
```bash
# 1. 安装必要系统包
sudo apt update
sudo apt install python3-venv python3-pip wget

# 2. 创建并验证虚拟环境
python3 -m venv .venv
if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
    echo "虚拟环境激活成功"
else
    echo "错误: 虚拟环境创建失败，请检查python3-venv是否安装"
    exit 1
fi

# 3. 安装Python依赖(指定ultralytics版本)
pip install opencv-python numpy pyserial
pip install ultralytics==8.0.196  # 指定稳定版本

# 4. 下载预训练模型(可选)
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt -O best.pt

# 5. 安装系统依赖(推荐)
sudo apt install libatlas3-base libopenjp2-7

# 6. 运行程序前激活环境
if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
else
    echo "错误: 虚拟环境未找到，请重新创建"
    exit 1
fi
```

## 使用说明
1. 启动主程序:
```bash
# 确保已激活虚拟环境
if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
else
    echo "错误: 虚拟环境未找到，请检查是否已创建"
    exit 1
fi
python main_controller.py
```

2. 校准线路颜色(可选):
```python
from line_follower import LineFollower
follower = LineFollower()
ret, frame = follower.cap.read()
follower.calibrate_color(frame)
```

3. 使用预训练模型:
```python
from camera_processor import CameraProcessor
processor = CameraProcessor()  # 自动加载best.pt或yolov8n.pt
```

## 性能优化建议
1. 使用散热器防止树莓派过热降频
2. 关闭图形界面释放资源:
```bash
sudo systemctl set-default multi-user.target
```
3. 使用`tmux`或`screen`后台运行程序
4. 降低检测分辨率提高帧率
