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
        }
    },
    "nbformat": 4,
    "nbformat_minor": 4
}


add_md("## 1. GPU, Sürümler ve Yapılandırma\nKaggle ortamında çalıştırılmak üzere GPU, path ve hiperparametre ayarları.")
add_code("""import torch
import torchvision
import os
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tqdm.auto import tqdm
from PIL import Image
import copy
import torchvision.transforms.functional as TF

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"PyTorch Version: {torch.__version__}")
print(f"Torchvision Version: {torchvision.__version__}")
print(f"Device: {device}")

# --- YAPIKANDIRMA (CONFIG) ---
DATASET_ROOT = '/kaggle/input/drone-road-segmentation-pilot'
OUT_DIR = '/kaggle/working/yol_segmentasyon_v1/'
os.makedirs(OUT_DIR, exist_ok=True)

# GPU belleği (OOM) koruması için 1920x1200 yerine orantı koruyan yarı-ölçek veya seçilen boyut
# 1920x1200 en-boy oranı: 1.6
TARGET_SIZE = (600, 960) # (Height, Width) 
BATCH_SIZE = 4
EPOCHS = 10
THRESHOLD = 0.5
RUN_TEST = False

print(f"Target Size: {TARGET_SIZE} (Aspect Ratio: {TARGET_SIZE[1]/TARGET_SIZE[0]:.2f})")
""")

add_md("## 2. Veri Okuma ve Bütünlük Doğrulaması\n\
Orijinal `3166 / 667 / 667` ayrımları hiçbir değişikliğe uğramadan yüklenir ve bağımsızlık kontrolü yapılır.")

add_code("""train_csv = os.path.join(DATASET_ROOT, 'train_pairs.csv')
val_csv = os.path.join(DATASET_ROOT, 'val_pairs.csv')
test_csv = os.path.join(DATASET_ROOT, 'test_pairs.csv')

df_t = pd.read_csv(train_csv)
df_v = pd.read_csv(val_csv)
df_te = pd.read_csv(test_csv)

print(f"Train samples: {len(df_t)} (Beklenen: 3166)")
print(f"Val samples  : {len(df_v)} (Beklenen: 667)")
print(f"Test samples : {len(df_te)} (Beklenen: 667)")

# Mutual exclusivity kontrol
set_t = set(df_t['rgb_path'])
set_v = set(df_v['rgb_path'])
set_te = set(df_te['rgb_path'])

assert len(set_t.intersection(set_v)) == 0, "HATA: Train ve Val arasında örtüşme var!"
assert len(set_t.intersection(set_te)) == 0, "HATA: Train ve Test arasında örtüşme var!"
assert len(set_v.intersection(set_te)) == 0, "HATA: Val ve Test arasında örtüşme var!"
print("Split kesişimsizlik doğrulandı.")
""")

add_md("## 3. Dataset ve Dataloader Sınıfı\n\
Boyut eşleşmesi, değer aralığı (0,1,2,3) ve nearest-neighbor maske yeniden boyutlandırma güvenlikleri.")

add_code("""from torch.utils.data import Dataset, DataLoader
import random

class RoadSegDataset(Dataset):
    def __init__(self, root_dir, dataframe, target_size, is_train=True):
        self.root_dir = root_dir
        self.df = dataframe
        self.target_size = target_size
        self.is_train = is_train
        
    def __len__(self):
        return len(self.df)
        
    def __getitem__(self, idx):
        row = self.df.iloc[idx]
        img_p = os.path.join(self.root_dir, row['rgb_path'])
        mask_p = os.path.join(self.root_dir, row['semantic_path'])
        
        # Dosya okuma ve güvenlik doğrulamaları
        img_bytes = np.fromfile(img_p, dtype=np.uint8)
        img = cv2.imdecode(img_bytes, cv2.IMREAD_COLOR)
        if img is None: raise ValueError(f"Okunamayan RGB: {img_p}")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        mask_bytes = np.fromfile(mask_p, dtype=np.uint8)
        raw_mask = cv2.imdecode(mask_bytes, cv2.IMREAD_UNCHANGED)
        if raw_mask is None: raise ValueError(f"Okunamayan Maske: {mask_p}")
            
        # Kanal kontrolü (Eğer 3 kanal ise R=G=B olmalı)
        if len(raw_mask.shape) == 3:
            b, g, r = raw_mask[:,:,0], raw_mask[:,:,1], raw_mask[:,:,2]
            if not (np.array_equal(b, g) and np.array_equal(g, r)):
                raise ValueError(f"Maske gri tonlamalı değil: {mask_p}")
            raw_mask = b
            
        h1, w1 = img.shape[:2]
        h2, w2 = raw_mask.shape[:2]
        if (h1, w1) != (h2, w2):
            raise ValueError(f"Boyut uyumsuzluğu! RGB {w1}x{h1}, Maske {w2}x{h2} -> {img_p}")
            
        unique_vals = np.unique(raw_mask)
        for v in unique_vals:
            if v not in [0, 1, 2, 3]:
                raise ValueError(f"Bilinmeyen etiket ({v}) maske dosyasında bulundu: {mask_p}")

        # BINARY MAPPING
        # Sadece 1 (Yol) hedeftir. Kalanı arkaplan(0) varsayılır. UNKNOWN(0) kenar kusurları kaynaktadır.
        bin_mask = (raw_mask == 1).astype(np.float32)
        
        img_pil = Image.fromarray(img)
        mask_pil = Image.fromarray((bin_mask * 255).astype(np.uint8))
        
        # Geometrik Transformlar (Maskeler her zaman NEAREST)
        img_pil = TF.resize(img_pil, self.target_size, interpolation=TF.InterpolationMode.BILINEAR)
        mask_pil = TF.resize(mask_pil, self.target_size, interpolation=TF.InterpolationMode.NEAREST)
        
        if self.is_train:
            if random.random() > 0.5:
                img_pil = TF.hflip(img_pil)
                mask_pil = TF.hflip(mask_pil)
                
        # Normalization
        img_t = TF.to_tensor(img_pil)
        img_t = TF.normalize(img_t, mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        
        mask_t = torch.from_numpy(np.array(mask_pil, dtype=np.float32) / 255.0).unsqueeze(0)
        
        return img_t, mask_t

train_ds = RoadSegDataset(DATASET_ROOT, df_t, TARGET_SIZE, is_train=True)
val_ds = RoadSegDataset(DATASET_ROOT, df_v, TARGET_SIZE, is_train=False)

train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True, pin_memory=True)
val_loader = DataLoader(val_ds, batch_size=BATCH_SIZE, shuffle=False)
""")

add_md("## 4. Eğitime Girecek RGB ve Maske Önizlemesi")
add_code("""try:
    sample_img, sample_mask = train_ds[0]

    img_view = sample_img.clone()
    mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
    img_view = (img_view * std) + mean
    img_view = img_view.permute(1, 2, 0).numpy()
    img_view = np.clip(img_view, 0, 1)

    mask_view = sample_mask[0].numpy()

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    axes[0].imshow(img_view)
    axes[0].set_title(f'Normalized RGB Input\\n{TARGET_SIZE}')

    axes[1].imshow(mask_view, cmap='gray')
    axes[1].set_title('Binary Mask (1=Road)')

    overlay = img_view.copy()
    overlay[mask_view > 0.5] = overlay[mask_view > 0.5] * 0.5 + np.array([0, 1, 0]) * 0.5
    axes[2].imshow(overlay)
    axes[2].set_title('Overlay')

    plt.show()
except Exception as e:
    print(f"Preview error, likely dataset not found in this environment: {e}")
""")

add_md("## 5. Model, Loss, Optimizer Tanımlama Fonksiyonu\n\
MobileNetV3 classifier çıkışları orijinal in_channels okunarak dinamik olarak adapte ediir.")

add_code("""from torchvision.models.segmentation import lraspp_mobilenet_v3_large, LRASPP_MobileNet_V3_Large_Weights
import torch.nn as nn
import torch.optim as optim

def setup_model_and_optimizer():
    model = lraspp_mobilenet_v3_large(weights=LRASPP_MobileNet_V3_Large_Weights.DEFAULT)
    
    # Mevcut giriş kanal sayılarını dinamik olarak belirle ve sadece çıktıları 1 yap
    low_ch = model.classifier.low_classifier.in_channels
    high_ch = model.classifier.high_classifier.in_channels
    
    model.classifier.low_classifier = nn.Conv2d(low_ch, 1, kernel_size=1)
    model.classifier.high_classifier = nn.Conv2d(high_ch, 1, kernel_size=1)
    
    model = model.to(device)
    
    criterion = nn.BCEWithLogitsLoss()
    optimizer = optim.Adam(model.parameters(), lr=1e-4)
    
    return model, criterion, optimizer

# Global TP accumulation function
def get_confusion_stats(pred_logits, masks, threshold=0.5):
    preds = (torch.sigmoid(pred_logits) > threshold).float()
    tp = (preds * masks).sum().item()
    fp = (preds * (1 - masks)).sum().item()
    fn = ((1 - preds) * masks).sum().item()
    tn = ((1 - preds) * (1 - masks)).sum().item()
    return tp, fp, fn, tn
""")


add_md("## 6. Sanity Check (Ön-Doğrulama)\n\
Eğitime başlamadan önce tek bir batch ile memory, shape loss limitlerini dener, ardından modeli sıfırlar.")
add_code("""try:
    print("--- SANITY CHECK BEGIN ---")
    chk_model, chk_crit, chk_opt = setup_model_and_optimizer()
    chk_model.train()
    
    # 1 batch çek
    imgs, masks = next(iter(train_loader))
    imgs, masks = imgs.to(device), masks.to(device)
    
    print(f"Input Shape: {imgs.shape}")
    print(f"Mask Shape: {masks.shape}")
    
    # Forward
    out = chk_model(imgs)['out']
    print(f"Output Shape: {out.shape}")
    assert out.shape == masks.shape, "Output ve mask shape eşleşmiyor!"
    
    # Backward
    loss = chk_crit(out, masks)
    print(f"Initial Loss (Finite Check): {loss.item():.4f}")
    assert torch.isfinite(loss), "Loss sonsuz (NaN/Inf) dönüyor!"
    
    chk_opt.zero_grad()
    loss.backward()
    chk_opt.step()
    
    # Clean up memory
    del chk_model, chk_crit, chk_opt, imgs, masks, out, loss
    torch.cuda.empty_cache()
    print("--- SANITY CHECK END (SUCCESS) ---")
except Exception as e:
    print(f"Sanity check error / skipped: {e}")
""")

add_md("## 7. Gerçek Eğitim ve Validation Döngüsü (Global Metrikler)")
add_code("""def validate_epoch(model, dataloader, criterion):
    model.eval()
    total_loss = 0.0
    G_TP, G_FP, G_FN = 0, 0, 0
    batches = 0
    
    with torch.no_grad():
        for imgs, masks in dataloader:
            imgs, masks = imgs.to(device), masks.to(device)
            out = model(imgs)['out']
            
            loss = criterion(out, masks)
            total_loss += loss.item()
            batches += 1
            
            tp, fp, fn, tn = get_confusion_stats(out, masks, THRESHOLD)
            G_TP += tp
            G_FP += fp
            G_FN += fn
            
    eps = 1e-6
    # Global piksel metrikleri (Batch ortalaması DEĞİL)
    iou = G_TP / (G_TP + G_FP + G_FN + eps)
    dice = 2 * G_TP / (2 * G_TP + G_FP + G_FN + eps)
    precision = G_TP / (G_TP + G_FP + eps)
    recall = G_TP / (G_TP + G_FN + eps)
    
    avg_loss = total_loss / max(batches, 1)
    
    return avg_loss, iou, dice, precision, recall

def train_segmentation():
    model, criterion, optimizer = setup_model_and_optimizer()
    
    config_metadata = {
        'model_base': 'lraspp_mobilenet_v3_large',
        'classes_mapped': '0,2,3 -> 0 (Background) | 1 -> 1 (Road)',
        'target_size': TARGET_SIZE,
        'normalization': {'mean': [0.485, 0.456, 0.406], 'std': [0.229, 0.224, 0.225]},
        'threshold': THRESHOLD,
        'batch_size': BATCH_SIZE
    }
    
    best_iou = -1.0 # ilk epoch'ta ne olursa olsun (sıfır bile olsa) kaydetsin
    
    print(f"Eğitim başlıyor... Toplam {EPOCHS} Epoch")
    for epoch in range(1, EPOCHS + 1):
        model.train()
        train_loss = 0.0
        b_count = 0
        
        # from tqdm.auto import tqdm
        # pbar = tqdm(train_loader, desc=f'Epoch {epoch}/{EPOCHS} [Train]')
        for imgs, masks in train_loader:
            imgs, masks = imgs.to(device), masks.to(device)
            
            optimizer.zero_grad()
            out = model(imgs)['out']
            loss = criterion(out, masks)
            
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item()
            b_count += 1
            # pbar.set_postfix({'loss': f"{loss.item():.4f}"})
            
        avg_train_loss = train_loss / max(b_count, 1)
        
        val_loss, val_iou, val_dice, val_prec, val_rec = validate_epoch(model, val_loader, criterion)
        
        print(f"Epoch [{epoch}/{EPOCHS}] | Train Loss: {avg_train_loss:.4f} | Val Loss: {val_loss:.4f}")
        print(f"--> Val IoU: {val_iou:.4f} | Val Dice: {val_dice:.4f} | Prec: {val_prec:.4f} | Rec: {val_rec:.4f}")
        
        # Checkpoint History Save
        if val_iou >= best_iou:
            best_iou = val_iou
            ckpt_path = os.path.join(OUT_DIR, 'best_road_seg_model.pth')
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'best_iou': best_iou,
                'config': config_metadata
            }, ckpt_path)
            print(f"*** YENI BEST IOU, CHECKPOINT KAYDEDILDI: {ckpt_path} ***")

# try:
#     train_segmentation()
# except Exception as e:
#     print(f"Kaggle/Dataset eksikliği nedeniyle eğitim koşulamadı: {e}")
""")

add_md("## 8. Test Kümesi Değerlendirmesi\n\
Validation metrikleriyle onaylanmış model, bağımsız test seti üzerinde çalıştırılır. Seçimleri kaydırmak için KULLANILAMAZ.")

add_code("""if RUN_TEST:
    test_ds = RoadSegDataset(DATASET_ROOT, df_te, TARGET_SIZE, is_train=False)
    test_loader = DataLoader(test_ds, batch_size=BATCH_SIZE, shuffle=False)
    
    eval_model, eval_crit, _ = setup_model_and_optimizer()
    ckpt_path = os.path.join(OUT_DIR, 'best_road_seg_model.pth')
    
    if os.path.exists(ckpt_path):
        ckpt = torch.load(ckpt_path, map_location=device)
        eval_model.load_state_dict(ckpt['model_state_dict'])
        print(f"Model yüklendi. Epoch: {ckpt['epoch']}, Val IoU'da: {ckpt['best_iou']:.4f}")
        
        test_loss, t_iou, t_dice, t_prec, t_rec = validate_epoch(eval_model, test_loader, eval_crit)
        print("====== TEST SET SONUCLARI ======")
        print(f"Loss: {test_loss:.4f}")
        print(f"IoU: {t_iou:.4f}")
        print(f"Dice: {t_dice:.4f}")
        print(f"Precision: {t_prec:.4f}")
        print(f"Recall: {t_rec:.4f}")
        print("================================")
    else:
        print("Checkpoint bulunamadı!")
else:
    print("Test değerlendirmesi RUN_TEST = False nedeniyle pas geçildi.")
""")

out_nb = r"c:\Users\Murat Ege\OneDrive\Masaüstü\DroneTest\QUADCOPTER_TEST\companion\yol_segmentasyon_v1.ipynb"
with open(out_nb, 'w', encoding='utf-8') as f:
    json.dump(nb, f, indent=2)

print(f"Notebook generated successfully: {out_nb}")
