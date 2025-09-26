# 端到端控制器插件

这是一个基于PyTorch模型的Nav2控制器插件，可以直接从传感器数据生成控制指令。

## 功能特性

- 使用LibTorch加载预训练的PyTorch模型（.pt格式）
- 输入数据：激光雷达扫描 + IMU数据 + 局部目标点
- 输出：线速度和角速度控制指令
- 完全兼容Nav2导航框架

## 文件结构

```
planner_config/
├── plan.hpp           # 头文件
├── plan.cpp           # 实现文件
└── plan.xml           # 插件描述文件

config/
└── end_to_end_controller.yaml  # 配置文件

scripts/
└── test_controller.py # 测试脚本

launch/
└── test_end_to_end.launch.py  # 测试launch文件
```

## 配置参数

在 `config/end_to_end_controller.yaml` 中：

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["EndToEndController"]
    EndToEndController:
      plugin: "zzzplan::EndToEndController"
      model_path: "/path/to/your/model.pt"  # PyTorch模型路径
      lookahead_distance: 0.3               # 前瞻距离(米)
      input_size: 364                       # 模型输入维度
      output_size: 2                        # 模型输出维度
```

## 模型要求

模型必须满足以下要求：
- **格式**: TorchScript (.pt文件)
- **输入**: 364维向量
  - 激光雷达数据: 360维
  - IMU数据: 2维 (角速度z, 线加速度x)
  - 局部目标点: 2维 (x, y)
- **输出**: 2维向量 (线速度, 角速度)

## 从Python模型转换为TorchScript

如果你有Python训练的模型(.pth文件)，需要转换为TorchScript格式：

```python
import torch

# 加载Python模型
model = YourModelClass(input_size=364, output_size=2)
model.load_state_dict(torch.load('model.pth'))
model.eval()

# 转换为TorchScript
traced_model = torch.jit.trace(model, torch.randn(1, 364))
traced_model.save('model.pt')
```

## 使用方法

1. **编译包**:
   ```bash
   cd your_ros2_workspace
   source /opt/ros/humble/setup.bash
   colcon build --packages-select car_nav
   ```

2. **更新模型路径**:
   编辑 `config/end_to_end_controller.yaml` 中的 `model_path`

3. **在Nav2中使用**:
   将此插件配置到你的Nav2参数文件中

4. **测试**:
   ```bash
   source install/setup.bash
   ros2 launch car_nav test_end_to_end.launch.py
   ```

## 订阅话题

- `/scan` (sensor_msgs/LaserScan): 激光雷达数据
- `/imu` (sensor_msgs/Imu): IMU数据
- 路径通过Nav2的setPlan()函数获取

## 发布话题

- 速度指令通过Nav2的computeVelocityCommands()返回

## 注意事项

1. 确保LibTorch已正确安装
2. 模型文件必须是TorchScript格式(.pt)
3. 输入维度必须与训练时一致
4. 传感器话题名称可能需要根据实际情况调整

## 故障排除

- **编译错误**: 检查LibTorch路径是否正确
- **模型加载失败**: 确认模型文件路径和格式
- **维度不匹配**: 检查激光雷达点数和模型input_size设置