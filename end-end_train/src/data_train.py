# src/train.py

import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# --- 1. 配置参数 ---
PROCESSED_DATA_DIR = Path('../processed_data')
MODEL_SAVE_PATH = Path('../model.pth')
NUM_EPOCHS = 60  # 训练轮数
BATCH_SIZE = 32  # 每批次训练的样本数
LEARNING_RATE = 0.001 # 学习率

# --- 2. 创建 PyTorch Dataset ---
# 这是告诉PyTorch如何加载我们数据的“说明书”
class DrivingDataset(Dataset):
    def __init__(self, data_dir):
        # 加载所有预处理好的数据
        self.scans = np.load(data_dir / 'scans.npy')
        self.imus = np.load(data_dir / 'imus.npy')
        self.goals = np.load(data_dir / 'goals.npy')
        self.actions = np.load(data_dir / 'actions.npy')
        
        # 将所有输入特征拼接在一起
        # shape: (样本数, 特征维度)
        self.features = np.hstack([self.scans, self.imus, self.goals]).astype(np.float32)
        self.labels = self.actions.astype(np.float32)

        self.num_samples = self.features.shape[0]
        print(f"Dataset loaded with {self.num_samples} samples.")

    def __len__(self):
        return self.num_samples

    def __getitem__(self, idx):
        # 根据索引返回一个样本
        return self.features[idx], self.labels[idx]

# --- 3. 设计神经网络模型 ---
# 这是模型的“大脑”结构
class DrivingModel(nn.Module):
    def __init__(self, input_size, output_size):
        super(DrivingModel, self).__init__()
        # 我们这里用一个简单的多层感知机 (MLP)
        self.network = nn.Sequential(
            nn.Linear(input_size, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, output_size)
        )

    def forward(self, x):
        return self.network(x)

# --- 4. 编写训练主逻辑 ---
def train():
    # 加载数据集
    full_dataset = DrivingDataset(PROCESSED_DATA_DIR)

    # 划分训练集和验证集 (90% 训练, 10% 验证)
    train_size = int(0.9 * len(full_dataset))
    val_size = len(full_dataset) - train_size
    train_dataset, val_dataset = random_split(full_dataset, [train_size, val_size])

    # 创建 DataLoader
    train_loader = DataLoader(dataset=train_dataset, batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(dataset=val_dataset, batch_size=BATCH_SIZE, shuffle=False)
    
    # 初始化模型
    input_size = full_dataset.features.shape[1]  # 特征维度
    output_size = full_dataset.labels.shape[1] # 动作维度 (线速度, 角速度)
    model = DrivingModel(input_size, output_size)
    print(f"Model initialized with input size {input_size} and output size {output_size}.")
    
    # 定义损失函数和优化器
    criterion = nn.MSELoss()  # 均方误差损失，适用于回归任务
    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
    
    train_losses, val_losses = [], []
    
    # 早停机制相关变量
    best_val_loss = float('inf')  # 记录最佳验证损失
    patience = 12  # 早停耐心值
    patience_counter = 0  # 耐心计数器
    
    print("--- Starting Training ---")
    # 训练循环
    for epoch in range(NUM_EPOCHS):
        model.train() # 设置为训练模式
        running_loss = 0.0
        for features, labels in train_loader:
            # 前向传播
            outputs = model(features)
            loss = criterion(outputs, labels)
            
            # 反向传播和优化
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            running_loss += loss.item()

        epoch_loss = running_loss / len(train_loader)
        train_losses.append(epoch_loss)
        
        # 验证过程
        model.eval() # 设置为评估模式
        val_loss = 0.0
        with torch.no_grad(): # 在评估时不计算梯度
            for features, labels in val_loader:
                outputs = model(features)
                loss = criterion(outputs, labels)
                val_loss += loss.item()
        
        val_loss /= len(val_loader)
        val_losses.append(val_loss)
        
        # 早停机制检查
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            patience_counter = 0
            # 保存最佳模型
            torch.save(model.state_dict(), MODEL_SAVE_PATH)
            print(f'Epoch [{epoch+1}/{NUM_EPOCHS}], Train Loss: {epoch_loss:.4f}, Validation Loss: {val_loss:.4f} (Best)')
        else:
            patience_counter += 1
            print(f'Epoch [{epoch+1}/{NUM_EPOCHS}], Train Loss: {epoch_loss:.4f}, Validation Loss: {val_loss:.4f} (Patience: {patience_counter}/{patience})')
            
            # 如果验证损失连续没有改善，提前停止训练
            if patience_counter >= patience:
                print(f"Early stopping at epoch {epoch+1} due to no improvement in validation loss")
                break

    print("--- Finished Training ---")
    
    # 如果没有触发早停，保存最终模型
    if patience_counter < patience:
        torch.save(model.state_dict(), MODEL_SAVE_PATH)
        print(f"Final model saved to {MODEL_SAVE_PATH}")
    else:
        print(f"Best model was already saved to {MODEL_SAVE_PATH}")
    
    # 绘制损失曲线
    plt.figure()
    plt.plot(train_losses, label='Train Loss')
    plt.plot(val_losses, label='Validation Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.legend()
    plt.title('Training and Validation Loss')
    plt.savefig('loss_curve.png')
    print("Loss curve saved to loss_curve.png")

if __name__ == '__main__':
    train()