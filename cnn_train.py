import os
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from collections import OrderedDict
from pathlib import Path
import pandas as pd
from PIL import Image
import torchvision.transforms as transforms
from torch.utils.tensorboard import SummaryWriter
from datetime import datetime
from multiprocessing import freeze_support
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import seaborn as sns

# Device configuration
device = 'cuda' if torch.cuda.is_available() else 'cpu'

# Hyperparameters
params = OrderedDict(
    lr=[0.001],
    batch_size=[32],
    num_workers=[4],
    device=device,
)

# --- Dataset for single thermal camera with steering and speed only ---
class ThermalDataset(Dataset):
    def __init__(self, file_path, image_dir, transform=None, image_ext=".jpg", indices=None):
        self.data = pd.read_csv(file_path)
        self.data = self.data.fillna(0)  # Fill NaN values with 0
        self.image_dir = image_dir
        
        self.image_paths = []
        self.steering_angles = []
        self.speeds = []

        print(f"Processing {len(self.data)} entries...")
        
        for index, row in self.data.iterrows():
            try:
                full_path = row['Image Name']
                filename = os.path.basename(full_path)
                img_path = os.path.join(image_dir, filename)
                
                if os.path.exists(img_path):
                    self.image_paths.append(img_path)
                    steering = float(row['Steering Angle'])
                    normalized_steering = steering / 0.7  # Normalize steering range to roughly [-1, 1]
                    self.steering_angles.append(normalized_steering)
                    self.speeds.append(float(row['Speed']))
                else:
                    print(f"Image not found: {img_path}")
            except Exception as e:
                print(f"Error processing row {index}: {e}")
                continue

        print(f"Successfully loaded {len(self.image_paths)} images")

        # Convert data to numpy arrays
        self.steering_angles = np.array(self.steering_angles, dtype=np.float32)
        self.speeds = np.array(self.speeds, dtype=np.float32)
        
        # Normalize speeds (adjust max_speed as needed)
        if len(self.speeds) > 0:
            max_speed = 1.5
            self.speeds = self.speeds / max_speed
        
        # If indices provided, then filter the data accordingly
        if indices is not None:
            self.image_paths = [self.image_paths[i] for i in indices]
            self.steering_angles = self.steering_angles[indices]
            self.speeds = self.speeds[indices]
        
        self.n_samples = len(self.image_paths)
        self.transform = transform

        print(f"Dataset initialized with {self.n_samples} samples")
        if self.n_samples > 0:
            print(f"Steering angle range: [{self.steering_angles.min():.2f}, {self.steering_angles.max():.2f}]")

    def __getitem__(self, index):
        img_path = self.image_paths[index]
        thermal_img = Image.open(img_path).convert('L')  # Grayscale for thermal
        thermal_img = thermal_img.crop((0, 0, 640, 430))

        if self.transform:
            image = self.transform(thermal_img)
        else:
            image = transforms.ToTensor()(thermal_img)

        # Create target tensor with steering and speed
        steering_angle = torch.tensor(self.steering_angles[index], dtype=torch.float32)
        speed = torch.tensor(self.speeds[index], dtype=torch.float32)
        targets = torch.tensor([steering_angle, speed], dtype=torch.float32)
        return image, targets

    def __len__(self):
        return self.n_samples

    def get_angle_distribution(self):
        return self.steering_angles

# --- Model for single thermal camera with two outputs ---
class ThermalEndToEnd(nn.Module):
    def __init__(self):
        super(ThermalEndToEnd, self).__init__()

        self.normalize = nn.BatchNorm2d(1)
        
        self.conv_layers = nn.Sequential(
            nn.Conv2d(1, 24, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.Conv2d(48, 64, kernel_size=3, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.Dropout2d(0.2)
        )
        
        self.calculate_conv_output_size()
        
        self.fc_shared = nn.Sequential(
            nn.Linear(self.fc_input_size, 100),
            nn.ReLU(),
            nn.Dropout(0.5)
        )
        
        # Separate heads for steering and speed predictions
        self.steering_head = nn.Sequential(
            nn.Linear(100, 50),
            nn.ReLU(),
            nn.Linear(50, 10),
            nn.ReLU(),
            nn.Linear(10, 1)
        )
        
        self.speed_head = nn.Sequential(
            nn.Linear(100, 50),
            nn.ReLU(),
            nn.Linear(50, 10),
            nn.ReLU(),
            nn.Linear(10, 1)
        )
        
        self._initialize_weights()

    def calculate_conv_output_size(self):
        # For an input of 640x512 (HxW reversed in torch as [C,H,W])
        x = torch.randn(1, 1, 430, 640)
        x = self.conv_layers(x)
        self.fc_input_size = np.prod(x.shape[1:])
        print(f"FC input size: {self.fc_input_size}")
    
    def _initialize_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.Linear):
                nn.init.normal_(m.weight, 0, 0.01)
                nn.init.constant_(m.bias, 0)

    def forward(self, x):
        x = self.normalize(x)
        x = self.conv_layers(x)
        x = x.view(x.size(0), -1)
        shared_features = self.fc_shared(x)
        steering = self.steering_head(shared_features)
        speed = self.speed_head(shared_features)
        return torch.cat((steering, speed), dim=1)

# --- Custom loss function with manual weights for two tasks ---
class ManualMultiTaskLoss(nn.Module):
    def __init__(self, w_steering=7.0, w_speed=1.0):
        """
        w_steering : weight for steering loss.
        w_speed    : weight for speed loss.
        """
        super(ManualMultiTaskLoss, self).__init__()
        self.steering_loss = nn.SmoothL1Loss()
        self.speed_loss = nn.MSELoss()
        self.w_steering = w_steering
        self.w_speed = w_speed

    def forward(self, outputs, targets):
        steering_output = outputs[:, 0].unsqueeze(1)
        speed_output = outputs[:, 1].unsqueeze(1)

        steering_target = targets[:, 0].unsqueeze(1)
        speed_target = targets[:, 1].unsqueeze(1)

        loss_steering = self.steering_loss(steering_output, steering_target)
        loss_speed = self.speed_loss(speed_output, speed_target)
        return self.w_steering * loss_steering + self.w_speed * loss_speed

# --- Checkpoint functions ---
def save_checkpoint(epoch, model, optimizer, loss, path):
    torch.save({
        'epoch': epoch,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
        'loss': loss,
    }, path)

def load_checkpoint(model, optimizer, path):
    checkpoint = torch.load(path)
    model.load_state_dict(checkpoint['model_state_dict'])
    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
    epoch = checkpoint['epoch']
    loss = checkpoint['loss']
    return epoch, loss

# --- Training and Validation Loops ---
def train_one_epoch(epoch_index, dataloader, writer, model, optimizer, loss_fn):
    model.train()
    running_loss = 0.0
    steering_correct = 0
    total = 0
    
    steering_threshold = 0.1  # Threshold for steering accuracy
    
    for i, (inputs, targets) in enumerate(dataloader):
        inputs = inputs.to(device)
        targets = targets.to(device)
        
        optimizer.zero_grad()
        outputs = model(inputs)
        loss = loss_fn(outputs, targets)
        loss.backward()
        
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        optimizer.step()
        
        running_loss += loss.item()
        tb_x = epoch_index * len(dataloader) + i
        writer.add_scalar('Loss/train', loss.item(), tb_x)
        
        # Calculate threshold-based steering accuracy
        steering_diff = torch.abs(outputs[:, 0] - targets[:, 0])
        steering_correct += torch.sum(steering_diff <= steering_threshold).item()
        total += targets.size(0)
        
        if i % 10 == 0:
            print(f'[Epoch {epoch_index}, Batch {i}] loss: {running_loss / (i + 1):.3f}, '
                  f'steering acc: {100 * steering_correct / total:.2f}%')
    
    avg_loss = running_loss / len(dataloader)
    steering_accuracy = 100 * steering_correct / total
    return avg_loss, steering_accuracy

def validate_one_epoch(dataloader, model, loss_fn):
    model.eval()
    running_loss = 0.0
    steering_correct = 0
    total = 0
    steering_threshold = 0.1
    
    steering_errors = []
    speed_errors = []
    
    with torch.no_grad():
        for inputs, targets in dataloader:
            inputs = inputs.to(device)
            targets = targets.to(device)
            outputs = model(inputs)
            loss = loss_fn(outputs, targets)
            running_loss += loss.item()
            
            steering_diff = torch.abs(outputs[:, 0] - targets[:, 0])
            steering_correct += torch.sum(steering_diff <= steering_threshold).item()
            steering_errors.extend(steering_diff.cpu().numpy())
            
            speed_diff = torch.abs(outputs[:, 1] - targets[:, 1])
            speed_errors.extend(speed_diff.cpu().numpy())
            
            total += targets.size(0)
    
    avg_loss = running_loss / len(dataloader)
    steering_accuracy = 100 * steering_correct / total
    mean_steering_error = np.mean(steering_errors)
    mean_speed_error = np.mean(speed_errors)
    
    return avg_loss, steering_accuracy, mean_steering_error, mean_speed_error

# --- Plot Functions ---
def plot_angle_distribution(dataset, save_dir="plots", filename="angle_distribution_thermal.pdf"):
    angles = dataset.get_angle_distribution()
    plt.figure()
    plt.hist(angles, bins=50, color='blue', alpha=0.7, edgecolor='black', density=True)
    plt.title('Steering Angle Distribution')
    plt.ylabel('Frequency')
    plt.grid(True, linestyle='--', alpha=0.5)
    sns.kdeplot(angles, color='red', linewidth=2, label='KDE')
    plt.legend()
    plt.tight_layout()
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, filename)
    plt.savefig(save_path, format='pdf', bbox_inches='tight')
    print(f"Angle distribution plot saved to {save_path}")
    plt.close()

def plot_metrics(train_losses, val_losses, steering_accuracies, save_dir="plots", filename="training_metrics_thermal.pdf"):
    plt.rcParams["figure.figsize"] = [12, 10]
    plt.rcParams["figure.autolayout"] = True
    fig, (ax1, ax2) = plt.subplots(2, 1)

    ax1.plot(train_losses, label='Training Loss')
    ax1.plot(val_losses, label='Validation Loss')
    ax1.set_xlabel('Epoch')
    ax1.set_ylabel('Loss')
    ax1.set_title('Training and Validation Losses')
    ax1.legend()
    ax1.grid(True)

    ax2.plot(steering_accuracies, label='Steering Accuracy')
    ax2.set_xlabel('Epoch')
    ax2.set_ylabel('Accuracy (%)')
    ax2.set_title('Steering Prediction Accuracy')
    ax2.legend()
    ax2.grid(True)

    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, filename)
    with PdfPages(save_path) as pdf:
        pdf.savefig(fig)
    print(f"Training metrics plot saved to {save_path}")
    plt.close()

def visualize_predictions(model, dataloader, num_samples=5, save_dir="plots", filename="prediction_visualization_thermal.pdf"):
    model.eval()
    fig, axs = plt.subplots(num_samples, 1, figsize=(10, num_samples * 4))
    if num_samples == 1:
        axs = [axs]
    
    with torch.no_grad():
        for i, (inputs, targets) in enumerate(dataloader):
            if i >= num_samples:
                break
            inputs = inputs.to(device)
            targets = targets.to(device)
            outputs = model(inputs)
            img = inputs[0].cpu().numpy()[0]  # First image, first channel
            steering_pred = outputs[0, 0].item()
            speed_pred = outputs[0, 1].item()
            steering_target = targets[0, 0].item()
            speed_target = targets[0, 1].item()
            axs[i].imshow(img, cmap='hot')
            axs[i].set_title(f"Sample {i+1}")
            prediction_text = (f"Steering: {steering_pred:.2f} (Target: {steering_target:.2f})\n"
                               f"Speed: {speed_pred:.2f} (Target: {speed_target:.2f})")
            axs[i].text(0.05, 0.95, prediction_text, transform=axs[i].transAxes,
                        verticalalignment='top', bbox={'boxstyle': 'round', 'alpha': 0.5})
            axs[i].axis('off')
    
    plt.tight_layout()
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, filename)
    plt.savefig(save_path, format='pdf', bbox_inches='tight')
    print(f"Prediction visualization saved to {save_path}")
    plt.close()

def analyze_model_outputs(model, dataloader, device, save_dir="plots", filename="model_output_distributions.pdf"):
    model.eval()
    steering_outputs = []
    speed_outputs = []
    
    with torch.no_grad():
        for inputs, targets in dataloader:
            inputs = inputs.to(device)
            outputs = model(inputs)
            steering_outputs.extend(outputs[:, 0].cpu().numpy())
            speed_outputs.extend(outputs[:, 1].cpu().numpy())
    
    print("Model Output Analysis:")
    print("Steering Outputs:")
    print(f"  Mean: {np.mean(steering_outputs)}")
    print(f"  Std Dev: {np.std(steering_outputs)}")
    print(f"  Min: {np.min(steering_outputs)}")
    print(f"  Max: {np.max(steering_outputs)}")
    
    print("\nSpeed Outputs:")
    print(f"  Mean: {np.mean(speed_outputs)}")
    print(f"  Std Dev: {np.std(speed_outputs)}")
    print(f"  Min: {np.min(speed_outputs)}")
    print(f"  Max: {np.max(speed_outputs)}")
    
    plt.figure(figsize=(15, 5))
    plt.subplot(121)
    plt.hist(steering_outputs, bins=50, color='blue', alpha=0.7)
    plt.title('Steering Output Distribution')
    plt.xlabel('Value')
    plt.ylabel('Frequency')
    
    plt.subplot(122)
    plt.hist(speed_outputs, bins=50, color='green', alpha=0.7)
    plt.title('Speed Output Distribution')
    plt.xlabel('Value')
    plt.ylabel('Frequency')
    
    plt.tight_layout()
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, filename)
    plt.savefig(save_path, format='pdf', bbox_inches='tight')
    print(f"Model output distribution plot saved to {save_path}")
    plt.close()

# --- Main Execution Logic ---
def main():
    print("Checking input paths...")
    file_path = '/home/avl/logging/logging_data/merged_output.csv'
    image_dir = '/home/avl/logging/image_data'
    
    print(f"CSV file exists: {os.path.exists(file_path)}")
    print(f"Image dir exists: {os.path.exists(image_dir)}")
    
    try:
        df = pd.read_csv(file_path)
        print(f"\nCSV head:\n{df.head()}")
        print(f"CSV shape: {df.shape}")
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return None, None, None

    # Define separate transforms for training and validation
    train_transform = transforms.Compose([
        transforms.RandomRotation(3),
        transforms.RandomAffine(0, translate=(0.05, 0.05)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.5], std=[0.5]),
    ])

    val_transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.5], std=[0.5]),
    ])

    # Load full dataset once to get the total number of samples and then split indices.
    full_temp_dataset = ThermalDataset(file_path=file_path, 
                                       image_dir=image_dir, 
                                       transform=None,   # No transform needed for splitting
                                       image_ext=".jpg")
    n = len(full_temp_dataset)
    indices = np.arange(n)
    np.random.shuffle(indices)
    split = int(0.8 * n)
    train_indices = indices[:split]
    val_indices = indices[split:]
    
    # Create two separate dataset instances with different transforms using the split indices
    train_dataset = ThermalDataset(file_path=file_path, 
                                   image_dir=image_dir, 
                                   transform=train_transform, 
                                   image_ext=".jpg",
                                   indices=train_indices)
    val_dataset = ThermalDataset(file_path=file_path, 
                                 image_dir=image_dir, 
                                 transform=val_transform, 
                                 image_ext=".jpg",
                                 indices=val_indices)
    
    print(f"\nTrain dataset size: {len(train_dataset)}")
    print(f"Validation dataset size: {len(val_dataset)}")
    
    # Optionally, inspect the first sample in the training set
    if len(train_dataset) > 0:
        first_img, first_target = train_dataset[0]
        print(f"\nFirst image shape: {first_img.shape}")
        print(f"First target values: {first_target}")
    
    train_dataloader = DataLoader(train_dataset, 
                                  batch_size=params['batch_size'][0],
                                  shuffle=True, 
                                  num_workers=params['num_workers'][0])
    val_dataloader = DataLoader(val_dataset, 
                                batch_size=params['batch_size'][0],
                                shuffle=False, 
                                num_workers=params['num_workers'][0])
    
    return train_dataloader, val_dataloader, train_dataset

def run():
    train_dataloader, val_dataloader, _ = main()
    
    if train_dataloader is None or val_dataloader is None:
        print("Failed to initialize datasets. Exiting...")
        return
    
    # Plot the steering angle distribution using the training dataset
    plot_angle_distribution(train_dataloader.dataset)
    
    # Initialize model and manual loss function
    model = ThermalEndToEnd().to(device)
    loss_fn = ManualMultiTaskLoss(w_steering=7.0, w_speed=1.0)
    # Use the same learning rate for model parameters; the loss function no longer has parameters.
    optimizer = optim.Adam(model.parameters(), lr=params['lr'][0], weight_decay=1e-4)
    
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode='min', factor=0.2, patience=5, verbose=True
    )

    timestamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    writer = SummaryWriter(f'runs/thermal_driver_{timestamp}')

    best_loss = float('inf')
    checkpoint_dir = Path("checkpoints")
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    checkpoint_path = checkpoint_dir / "best_model_thermal.pth"
    
    start_epoch = 0
    completed_epoch = 0
    train_losses = []
    val_losses = []
    steering_accuracies = []
    steering_errors = []
    speed_errors = []
    max_epochs = 50

    if checkpoint_path.exists():
        print(f"Loading checkpoint from {checkpoint_path}...")
        checkpoint = torch.load(checkpoint_path)
        model.load_state_dict(checkpoint['model_state_dict'])
        optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        start_epoch = checkpoint['epoch'] + 1
        best_loss = checkpoint['loss']
        print(f"Resuming training from epoch {start_epoch} with loss {best_loss:.4f}")

    try:
        for epoch in range(start_epoch, max_epochs):
            print(f'Epoch {epoch + 1}/{max_epochs}')
            completed_epoch = epoch

            train_loss, train_steering_acc = train_one_epoch(
                epoch, train_dataloader, writer, model, optimizer, loss_fn
            )
            
            val_loss, val_steering_acc, mean_steering_error, mean_speed_error = validate_one_epoch(
                val_dataloader, model, loss_fn
            )
            
            train_losses.append(train_loss)
            val_losses.append(val_loss)
            steering_accuracies.append(val_steering_acc)
            steering_errors.append(mean_steering_error)
            speed_errors.append(mean_speed_error)

            writer.add_scalar('Loss/val_avg', val_loss, epoch)
            writer.add_scalar('Accuracy/steering', val_steering_acc, epoch)
            writer.add_scalar('Error/steering', mean_steering_error, epoch)
            writer.add_scalar('Error/speed', mean_speed_error, epoch)
            
            print(f"Epoch {epoch + 1} - Train Loss: {train_loss:.4f}, Val Loss: {val_loss:.4f}")
            print(f"Steering Acc: {val_steering_acc:.2f}%")
            print(f"Mean Steering Error: {mean_steering_error:.4f}, Mean Speed Error: {mean_speed_error:.4f}")
            
            if val_loss < best_loss:
                best_loss = val_loss
                save_checkpoint(epoch, model, optimizer, val_loss, checkpoint_path)
                print(f"New best model saved with validation loss: {best_loss:.4f}")

            scheduler.step(val_loss)
    except Exception as e:
        print(f"Training interrupted: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        final_model_path = checkpoint_dir / "final_model_thermal.pth"
        torch.save({
            'epoch': completed_epoch,
            'model_state_dict': model.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
            'loss': val_losses[-1] if val_losses else float('inf'),
        }, final_model_path)
        print(f"Final model saved at {final_model_path}")

        writer.close()

        if train_losses:
            metrics_data = pd.DataFrame({
                'epoch': range(start_epoch, completed_epoch + 1),
                'train_loss': train_losses,
                'val_loss': val_losses,
                'steering_accuracy': steering_accuracies,
                'steering_error': steering_errors,
                'speed_error': speed_errors
            })
            metrics_data.to_csv('training_metrics_thermal.csv', index=False)
            print("Training metrics saved to training_metrics_thermal.csv")
            plot_metrics(train_losses, val_losses, steering_accuracies)
            
            print("Loading best model for prediction visualization...")
            best_model = ThermalEndToEnd().to(device)
            checkpoint = torch.load(checkpoint_path)
            best_model.load_state_dict(checkpoint['model_state_dict'])
            visualize_predictions(best_model, val_dataloader, num_samples=5)

        print("Training complete!")

if __name__ == '__main__':
    freeze_support()
    run()
