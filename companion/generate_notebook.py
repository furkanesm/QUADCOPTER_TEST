import json
import os

cells = []

def add_md(text):
    cells.append({
        "cell_type": "markdown",
        "metadata": {},
        "source": [line + "\n" for line in text.split('\n')]
    })

def add_code(text):
    cells.append({
        "cell_type": "code",
        "execution_count": None,
        "metadata": {},
        "outputs": [],
        "source": [line + "\n" if i < len(text.split('\n')) - 1 else line for i, line in enumerate(text.split('\n'))]
    })

nb = {
    "cells": cells,
    "metadata": {
        "kernelspec": {
            "display_name": "Python 3",
            "language": "python",
            "name": "python3"
        },
        "language_info": {
            "codemirror_mode": {"name": "ipython", "version": 3},
            "file_extension": ".py",
            "mimetype": "text/x-python",
            "name": "python",
            "nbconvert_exporter": "python",
            "pygments_lexer": "ipython3",
            "version": "3.10.12"
        }
    },
    "nbformat": 4,
    "nbformat_minor": 4
}


add_md("## 1. GPU, Sürümler ve Veri Kökü")
add_code("""import torch
import torchvision
import os
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tqdm.auto import tqdm
from PIL import Image

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"PyTorch Version: {torch.__version__}")
print(f"Torchvision Version: {torchvision.__version__}")
print(f"Device: {device}")

DATASET_ROOT = '/kaggle/input/drone-road-segmentation-pilot' # kaggle input varsayımı
""")

add_md("## 2. CSV Okuma, Dosya/Sınıf Doğrulaması\n\n\
**DİKKAT: SINIF NUMARALARI GRIDPLANNER İLE AYNI DEĞİLDİR!**\n\
GridPlanner Sınıfları (0=FREE, 1=OBSTACLE, 2=UNKNOWN) İKEN,\n\
Eğitim Kaynak Maskesi Şöyledir:\n\
1 = Yol (Eğitim pozitif hedefi = 1 olacak)\\n\
0, 2, 3 = UNKNOWN, Engel, Hedef (Eğitim arka planı = 0 olacak)")

add_code("""train_csv = os.path.join(DATASET_ROOT, 'train_pairs.csv')
val_csv = os.path.join(DATASET_ROOT, 'val_pairs.csv')

df_train = pd.read_csv(train_csv)
df_val = pd.read_csv(val_csv)

print(f"Train samples: {len(df_train)}")
print(f"Val samples: {len(df_val)}")

# Basic existence check
for _, row in df_train.head(3).iterrows():
    img_path = os.path.join(DATASET_ROOT, row['rgb_path'])
    mask_path = os.path.join(DATASET_ROOT, row['semantic_path'])
    assert os.path.exists(img_path), f"Missing: {img_path}"
    assert os.path.exists(mask_path), f"Missing: {mask_path}"
print("File checks passed for first rows.")
""")

add_md("## 3. RGB+Maske Veri Yükleyici ve Geometrik Dönüşümler\n\
Görüntü ve maskeler üzerinde KESİNLİKLE aynı geometrik rastgele dönüşümler uygulanmalı.\n\
Maske için yalnızca `nearest-neighbor` veya raw crop/flip kullanılmalıdır.")

add_code("""from torch.utils.data import Dataset, DataLoader
import torchvision.transforms.functional as TF
import random

class RoadSegDataset(Dataset):
    def __init__(self, root_dir, dataframe, is_train=True):
        self.root_dir = root_dir
        self.df = dataframe
        self.is_train = is_train
        
    def __len__(self):
        return len(self.df)
        
    def __getitem__(self, idx):
        row = self.df.iloc[idx]
        img_p = os.path.join(self.root_dir, row['rgb_path'])
        mask_p = os.path.join(self.root_dir, row['semantic_path'])
        
        # Load RGB
        img_bytes = np.fromfile(img_p, dtype=np.uint8)
        img = cv2.imdecode(img_bytes, cv2.IMREAD_COLOR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Load Mask
        mask_bytes = np.fromfile(mask_p, dtype=np.uint8)
        raw_mask = cv2.imdecode(mask_bytes, cv2.IMREAD_UNCHANGED)
        if len(raw_mask.shape) == 3:
            raw_mask = raw_mask[:,:,0] # Take one channel
            
        # BINARY MAPPING
        # Sadece 1 (Yol) piksellerini 1 yapıyoruz. 0 (UNKNOWN), 2 ve 3 pikselleri arka plan 0'a karışır.
        bin_mask = (raw_mask == 1).astype(np.float32)
        
        # Convert to PIL for identical torchvision functional transforms
        img_pil = Image.fromarray(img)
        mask_pil = Image.fromarray((bin_mask * 255).astype(np.uint8))
        
        # Identical Geometric Transforms
        if self.is_train:
            if random.random() > 0.5:
                img_pil = TF.hflip(img_pil)
                mask_pil = TF.hflip(mask_pil)
                
            # Random Crop could go here, omitting complex rotations to preserve camera dynamics stability
            
        # Image base transforms
        img_t = TF.to_tensor(img_pil)
        img_t = TF.normalize(img_t, mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        
        # Mask tensor must stay exact (0, 1), NEAREST inherently applied to raw pixel values during tensorisation
        mask_t = torch.from_numpy(np.array(mask_pil, dtype=np.float32) / 255.0).unsqueeze(0)
        
        return img_t, mask_t

train_ds = RoadSegDataset(DATASET_ROOT, df_train, is_train=True)
val_ds = RoadSegDataset(DATASET_ROOT, df_val, is_train=False)

train_loader = DataLoader(train_ds, batch_size=4, shuffle=True, pin_memory=True)
val_loader = DataLoader(val_ds, batch_size=4, shuffle=False)
""")

add_md("## 4. Eğitime Girecek RGB ve Maske Önizlemesi (Görsel Kontrol)")
add_code("""sample_img, sample_mask = train_ds[0]

# Denormalize for viewing
img_view = sample_img.clone()
mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
img_view = (img_view * std) + mean
img_view = img_view.permute(1, 2, 0).numpy()
img_view = np.clip(img_view, 0, 1)

mask_view = sample_mask[0].numpy()

fig, axes = plt.subplots(1, 3, figsize=(15, 5))
axes[0].imshow(img_view)
axes[0].set_title('Normalized RGB Input')

axes[1].imshow(mask_view, cmap='gray')
axes[1].set_title('Binary Road Mask (1=Road)')

overlay = img_view.copy()
overlay[mask_view > 0.5] = overlay[mask_view > 0.5] * 0.5 + np.array([0, 1, 0]) * 0.5
axes[2].imshow(overlay)
axes[2].set_title('Overlay (Green=Road)')

plt.show()
""")

add_md("## 5. Model, Loss, Optimizer\n\
PyTorch MobileNetV3 tabanlı hafif LRASPP segmentasyon modelini kullanıyoruz.\n\
(Not: Sadece ikili sınıf (Road vs BG) olduğu için BCEWithLogits kullanacağız, çıktı kanalı num_classes=1 olacak.)")

add_code("""from torchvision.models.segmentation import lraspp_mobilenet_v3_large, LRASPP_MobileNet_V3_Large_Weights
import torch.nn as nn
import torch.optim as optim

# Default model is trained on COCO with 21 classes. We inject num_classes=1 for BCE
model = lraspp_mobilenet_v3_large(weights=LRASPP_MobileNet_V3_Large_Weights.DEFAULT)

# Replace the classifier for 1-channel binary segmentation output
model.classifier.low_classifier = nn.Conv2d(40, 1, kernel_size=1)
model.classifier.high_classifier = nn.Conv2d(128, 1, kernel_size=1)
model = model.to(device)

criterion = nn.BCEWithLogitsLoss() # Automatically applies Sigmoid, targets are 0/1
optimizer = optim.Adam(model.parameters(), lr=1e-4)
""")


add_md("## 6. Eğitim Loop (Configurable 10 Epoch)")
add_code("""EPOCHS = 10
best_iou = 0.0
os.makedirs('/kaggle/working/checkpoints', exist_ok=True)

# Sınıf eşleme parametrelerini checkpoint'e sabitle!
config_metadata = {
    'input_classes_handled': {
        'road': 'Mapped to 1',
        'unknown': 'Mapped to 0',
        'obstacle': 'Mapped to 0',
        'target': 'Mapped to 0'
    },
    'model_base': 'lraspp_mobilenet_v3_large'
}

def calculate_metrics(pred_logits, masks, threshold=0.5):
    preds = (torch.sigmoid(pred_logits) > threshold).float()
    
    tp = (preds * masks).sum().item()
    fp = (preds * (1 - masks)).sum().item()
    fn = ((1 - preds) * masks).sum().item()
    tn = ((1 - preds) * (1 - masks)).sum().item()
    
    eps = 1e-6
    iou = tp / (tp + fp + fn + eps)
    dice = 2 * tp / (2 * tp + fp + fn + eps)
    precision = tp / (tp + fp + eps)
    recall = tp / (tp + fn + eps)
    
    return iou, dice, precision, recall

# Train loop placeholder logic:
# for epoch in range(EPOCHS):
#    model.train()
#    for imgs, masks in train_loader:
#        ...
""")

add_md("## 7. Validation: IoU, Dice, Precision, Recall metrikleriyle")
add_code("""# Validation Loop Function
def validate_epoch(val_dataloader):
    model.eval()
    total_iou, total_dice, total_prec, total_rec = 0, 0, 0, 0
    batches = 0
    
    with torch.no_grad():
        # for imgs, masks in tqdm(val_dataloader, desc='Validation'):
        #     imgs, masks = imgs.to(device), masks.to(device)
        #     out = model(imgs)['out']
        #     iou, dice, prec, rec = calculate_metrics(out, masks)
        #     total_iou += iou; total_dice += dice; total_prec += prec; total_rec += rec
        #     batches += 1
        pass
        
    # return total_iou/max(batches,1), total_dice/max(batches,1), total_prec/max(batches,1), total_rec/max(batches,1)
""")

add_md("## 8. Best Checkpoint Save (IoU Based)")
add_code("""# Training simulation check (Logic)
'''
val_iou, val_dice, val_prec, val_rec = validate_epoch(val_loader)
print(f"Epoch {epoch} | Val IoU (Road): {val_iou:.4f} | Val Dice: {val_dice:.4f}")

if val_iou > best_iou:
    best_iou = val_iou
    ckpt_path = '/kaggle/working/checkpoints/best_road_seg_model.pth'
    torch.save({
        'epoch': epoch,
        'model_state_dict': model.state_dict(),
        'best_iou': best_iou,
        'config': config_metadata
    }, ckpt_path)
    print(f"Saved new best model to {ckpt_path}")
'''
""")

add_md("## 9. Test Kümesi Değerlendirme (Varsayılan Çalışmayan Seçim)")
add_code("""# DİKKAT: Test kümesi hiperparametre/model/threshold ince ayarı için KESİNLİKLE GÖRÜLMEYECEK.
# Bu hücre yalnızca en son, bitti dendiği aşamada saf başarı raporlaması için çalıştırılacaktır.
# Asla eğitim esnasında best weight seçimine sokmayınız.

'''
test_csv = os.path.join(DATASET_ROOT, 'test_pairs.csv')
df_test = pd.read_csv(test_csv)
test_ds = RoadSegDataset(DATASET_ROOT, df_test, is_train=False)
test_loader = DataLoader(test_ds, batch_size=4, shuffle=False)

# load best model
# model.load_state_dict(torch.load('/kaggle/working/checkpoints/best_road_seg_model.pth')['model_state_dict'])
# test_iou, test_dice, test_prec, test_rec = validate_epoch(test_loader)
# print(f"--- TEST SET EVALUATION ---")
# print(f"IoU: {test_iou:.4f} | Dice: {test_dice:.4f} | Precision: {test_prec:.4f} | Recall: {test_rec:.4f}")
'''
""")

out_nb = r"c:\Users\Murat Ege\OneDrive\Masaüstü\DroneTest\QUADCOPTER_TEST\companion\yol_segmentasyon_v1.ipynb"
with open(out_nb, 'w', encoding='utf-8') as f:
    json.dump(nb, f, indent=2)

print(f"Notebook generated: {out_nb}")
