# src/export_model.py (CNN Version)

import torch
# 确保从包含新CNN模型定义的 data_train.py 文件中导入
from data_train import DrivingModel 

# --- 配置 (修改点 1: 更新模型输入参数) ---
MODEL_PATH = '../cnn_model_0.5s.pth' # 确保这是你训练好的CNN模型的路径
EXPORT_PATH = '../cnn_model_0.5s.pt'  # 导出的 Torch Script 模型路径

# 新模型需要知道激光雷达的点数和辅助特征的数量
# 请根据你 data_train.py 中的实际情况修改这些值
NUM_SCAN_POINTS = 360 # 假设你的激光雷达有360个点
NUM_AUX_FEATURES = 4  # IMU(2) + Goal(2) = 4个辅助特征
OUTPUT_SIZE = 2       # 输出保持不变 (linear_x, angular_z)

def export():
    # 1. 初始化模型结构 (修改点 2: 使用新的初始化参数)
    print("Initializing model structure...")
    model = DrivingModel(
        num_scan_points=NUM_SCAN_POINTS, 
        num_aux_features=NUM_AUX_FEATURES, 
        output_size=OUTPUT_SIZE
    )
    
    # 2. 加载已经训练好的权重
    print(f"Loading weights from {MODEL_PATH}...")
    # 使用 map_location='cpu' 确保模型可以在没有GPU的环境下加载
    model.load_state_dict(torch.load(MODEL_PATH, map_location=torch.device('cpu')))
    
    # 3. 将模型设置为评估模式
    model.eval()
    print("Model is in evaluation mode.")
    
    # 4. 创建符合模型输入的示例张量 (修改点 3: 创建两个独立的示例输入)
    #    维度必须与模型训练时完全一致
    #    输入1: 给CNN的激光雷达数据 (batch_size, channels, num_points)
    example_scan_input = torch.randn(1, 1, NUM_SCAN_POINTS)
    #    输入2: 给MLP的辅助特征数据 (batch_size, num_features)
    example_aux_input = torch.randn(1, NUM_AUX_FEATURES)
    print(f"Created example scan input with shape: {example_scan_input.shape}")
    print(f"Created example auxiliary input with shape: {example_aux_input.shape}")

    
    # 5. 使用 torch.jit.trace 进行转换 (修改点 4: 将两个示例输入打包成一个元组传入)
    print("Tracing model with torch.jit.trace...")
    # 将两个示例输入放入一个元组 (tuple) 中
    traced_script_module = torch.jit.trace(model, (example_scan_input, example_aux_input))
    
    # 6. 保存转换后的模型
    traced_script_module.save(EXPORT_PATH)
    
    print(f"\nModel successfully converted from {MODEL_PATH} to {EXPORT_PATH}")
    print("You can now use this .pt file in C++ or other deployment environments.")

if __name__ == '__main__':
    export()