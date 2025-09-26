# src/export_model.py

import torch
from data_train import DrivingModel # 从我们之前的训练脚本中导入模型定义

# --- 配置 ---
# 选择你要转换的那个 .pth 模型文件
MODEL_PATH = '../model_0.5s.pth' 
# 你需要知道模型的输入维度
INPUT_SIZE = 364
OUTPUT_SIZE = 2
# 输出的 Torch Script 模型路径
EXPORT_PATH = '../model_0.5s.pt' 

def export():
    # 1. 初始化模型结构
    model = DrivingModel(INPUT_SIZE, OUTPUT_SIZE)
    
    # 2. 加载已经训练好的权重
    # 我们需要指定 map_location=torch.device('cpu')，以防模型是在GPU上训练的
    model.load_state_dict(torch.load(MODEL_PATH, map_location=torch.device('cpu')))
    
    # 3. 将模型设置为评估模式
    model.eval()
    
    # 4. 创建一个符合模型输入的示例张量
    # 维度必须是 (1, input_size)，1代表batch_size
    example_input = torch.randn(1, INPUT_SIZE)
    
    # 5. 使用 torch.jit.trace 进行转换
    traced_script_module = torch.jit.trace(model, example_input)
    
    # 6. 保存转换后的模型
    traced_script_module.save(EXPORT_PATH)
    
    print(f"Model successfully converted from {MODEL_PATH} to {EXPORT_PATH}")

if __name__ == '__main__':
    export()