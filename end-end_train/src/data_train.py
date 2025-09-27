# src/train.py (CNN Version)

import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# --- 1. 配置参数 (保持不变) ---
PROCESSED_DATA_DIR = Path('../processed_data')
MODEL_SAVE_PATH = Path('../cnn_model_0.5s.pth') # 建议为新模型使用新名称
NUM_EPOCHS = 60
BATCH_SIZE = 32
LEARNING_RATE = 0.001
PATIENCE = 12 # 早停耐心值

# --- 2. 创建 PyTorch Dataset (重大修改) ---
# 这个版本将激光雷达数据和其他辅助数据分开处理
class DrivingDataset(Dataset):
    def __init__(self, data_dir):
        # 分别加载数据
        scans_raw = np.load(data_dir / 'scans.npy')
        imus = np.load(data_dir / 'imus.npy')
        goals = np.load(data_dir / 'goals.npy')
        
        # 1. 预处理激光雷达数据以适配CNN
        #    替换无穷大(inf)和非数字(nan)的值，这是很重要的预处理步骤
        scans_raw[np.isinf(scans_raw)] = 10.0  # 用一个大的固定值（例如雷达最大量程）替换inf
        scans_raw[np.isnan(scans_raw)] = 0.0   # 用0替换nan
        
        #    为CNN增加一个“通道”维度 (这是关键)
        #    PyTorch的Conv1d需要 (N, Channels, Length) 格式的输入
        #    我们将 (样本数, 激光点数) 变形为 (样本数, 1, 激光点数)
        self.scans = np.expand_dims(scans_raw, axis=1).astype(np.float32)

        # 2. 拼接IMU和Goal等不具备空间特征的辅助数据
        self.aux_features = np.hstack([imus, goals]).astype(np.float32)

        # 3. 加载标签 (控制指令)
        self.labels = np.load(data_dir / 'actions.npy').astype(np.float32)

        self.num_samples = self.scans.shape[0]
        print(f"Dataset loaded with {self.num_samples} samples.")
        print(f"Scan shape for CNN: {self.scans.shape}")
        print(f"Auxiliary features shape: {self.aux_features.shape}")

    def __len__(self):
        return self.num_samples

    def __getitem__(self, idx):
        # 返回三个独立的部分：激光数据，辅助数据，标签
        return self.scans[idx], self.aux_features[idx], self.labels[idx]

# --- 3. 设计神经网络模型 (完全重写为CNN模型) ---
# 这是一个混合模型：CNN分支处理激光数据，然后与辅助数据融合，最后通过MLP输出
class DrivingModel(nn.Module):
    def __init__(self, num_scan_points, num_aux_features, output_size):
        super(DrivingModel, self).__init__()
        
        # CNN 分支 - 用于从激光雷达数据中提取空间特征
        self.cnn_branch = nn.Sequential(
            # 输入形状: (Batch, 1, num_scan_points)
            nn.Conv1d(in_channels=1, out_channels=32, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2, stride=2),
            # 输出形状: (Batch, 32, ...)
            nn.Conv1d(in_channels=32, out_channels=64, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2, stride=2),
            # 输出形状: (Batch, 64, ...)
            nn.Flatten() # 将卷积后的特征图展平成一维向量
        )
        
        # 使用一个虚拟输入来自动计算CNN分支展平后的输出维度
        with torch.no_grad():
            dummy_input = torch.zeros(1, 1, num_scan_points)
            cnn_output_size = self.cnn_branch(dummy_input).shape[1]
            print(f"CNN flattened output size: {cnn_output_size}")
            
        # 融合与输出分支 (MLP) - 结合所有特征并输出最终指令
        self.fusion_branch = nn.Sequential(
            # 输入维度 = CNN输出维度 + 辅助特征维度
            nn.Linear(cnn_output_size + num_aux_features, 128),
            nn.ReLU(),
            nn.Dropout(0.5), # 添加Dropout防止过拟合
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, output_size)
        )

    def forward(self, scan_input, aux_input):
        # 1. 激光数据通过CNN分支
        scan_features = self.cnn_branch(scan_input)
        
        # 2. 将CNN提取的特征与辅助特征在维度1上拼接
        combined_features = torch.cat((scan_features, aux_input), dim=1)
        
        # 3. 通过最后的MLP层得到输出
        output = self.fusion_branch(combined_features)
        return output

# --- 4. 编写训练主逻辑 (微小但关键的修改) ---
def train():
    # 设置设备
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    # 加载数据集
    full_dataset = DrivingDataset(PROCESSED_DATA_DIR)

    # 划分训练集和验证集
    train_size = int(0.9 * len(full_dataset))
    val_size = len(full_dataset) - train_size
    train_dataset, val_dataset = random_split(full_dataset, [train_size, val_size])

    # 创建 DataLoader
    train_loader = DataLoader(dataset=train_dataset, batch_size=BATCH_SIZE, shuffle=True, num_workers=4)
    val_loader = DataLoader(dataset=val_dataset, batch_size=BATCH_SIZE, shuffle=False, num_workers=4)
    
    # 初始化模型 (根据新Dataset的结构)
    num_scan_points = full_dataset.scans.shape[2]
    num_aux_features = full_dataset.aux_features.shape[1]
    output_size = full_dataset.labels.shape[1]
    
    model = DrivingModel(num_scan_points, num_aux_features, output_size).to(device)
    
    # 定义损失函数和优化器
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
    
    train_losses, val_losses = [], []
    
    # 早停机制相关变量
    best_val_loss = float('inf')
    patience_counter = 0
    
    print("--- Starting Training ---")
    for epoch in range(NUM_EPOCHS):
        model.train()
        running_loss = 0.0
        # (修改点) 从DataLoader中解包出三项
        for scans, aux_features, labels in train_loader:
            scans = scans.to(device)
            aux_features = aux_features.to(device)
            labels = labels.to(device)
            
            # 前向传播 (修改点) 向模型传入两个输入
            outputs = model(scans, aux_features)
            loss = criterion(outputs, labels)
            
            # 反向传播和优化
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            running_loss += loss.item()

        epoch_loss = running_loss / len(train_loader)
        train_losses.append(epoch_loss)
        
        # 验证过程
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            # (修改点) 从DataLoader中解包出三项
            for scans, aux_features, labels in val_loader:
                scans = scans.to(device)
                aux_features = aux_features.to(device)
                labels = labels.to(device)
                
                # (修改点) 向模型传入两个输入
                outputs = model(scans, aux_features)
                loss = criterion(outputs, labels)
                val_loss += loss.item()
        
        val_loss /= len(val_loader)
        val_losses.append(val_loss)
        
        # 早停机制和模型保存
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            patience_counter = 0
            torch.save(model.state_dict(), MODEL_SAVE_PATH)
            print(f'Epoch [{epoch+1}/{NUM_EPOCHS}], Train Loss: {epoch_loss:.4f}, Val Loss: {val_loss:.4f} (Best model saved)')
        else:
            patience_counter += 1
            print(f'Epoch [{epoch+1}/{NUM_EPOCHS}], Train Loss: {epoch_loss:.4f}, Val Loss: {val_loss:.4f} (Patience: {patience_counter}/{PATIENCE})')
            if patience_counter >= PATIENCE:
                print(f"Early stopping at epoch {epoch+1}.")
                break

    print("--- Finished Training ---")
    print(f"Best model saved to {MODEL_SAVE_PATH}")
    
    # 绘制损失曲线
    plt.figure()
    plt.plot(train_losses, label='Train Loss')
    plt.plot(val_losses, label='Validation Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.legend()
    plt.title('Training and Validation Loss (CNN Model)')
    plt.savefig('cnn_loss_curve.png')
    print("Loss curve saved to cnn_loss_curve.png")

if __name__ == '__main__':
    train()