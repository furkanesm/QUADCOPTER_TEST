"""
##############################################################################
# KAGGLE SEGMENTATION NOTEBOOK SKELETON
##############################################################################

DİKKAT: SINIF İSİMLENDİRME VE NUMARALANDIRMA FARKINDALIĞI!
Bu eğitimde Unity'den gelen HAM semantic maskeler kullanılmaktadır.

[KAYNAK UNITY MASKE SINIFLARI (Eğitimde bunlar geçerli)]:
0 = UNKNOWN / Etiketlenmemiş Arka Plan
1 = Yol (FREE)
2 = Engel (OBSTACLE)
3 = Hedef (TARGET)

[GRID_PATH_PLANNER İÇ ALGORİTMA SINIFLARI (Eğitimde KULLANILMAYACAK)]:
0 = FREE (Yol)
1 = OBSTACLE (Engel)
2 = UNKNOWN

Bu nedenle Dataloader içinde piksel değerleri (0,1,2,3) orijinal halleriyle kalmalı, 
eğitim sonrası veya route extraction aşamasında GridPlanner'a aktarılırken remap 
işlemi plan_route_from_mask.py'deki gibi (1->0, 2->1, 0->2) yapılmalıdır.
"""

import os
import csv
import torch
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
import numpy as np
import cv2

class DroneSegmentationDataset(Dataset):
    def __init__(self, csv_file, transform=None, mask_transform=None, mapping_strategy="Option1"):
        """
        csv_file: train_pairs.csv, val_pairs.csv, test_pairs.csv
        """
        self.pairs = []
        with open(csv_file, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.pairs.append((row['rgb_path'], row['semantic_path']))
                
        self.transform = transform
        self.mapping_strategy = mapping_strategy
        
    def __len__(self):
        return len(self.pairs)
        
    def __getitem__(self, idx):
        rgb_path, mask_path = self.pairs[idx]
        
        # Load RGB
        img_bytes = np.fromfile(rgb_path, dtype=np.uint8)
        img = cv2.imdecode(img_bytes, cv2.IMREAD_COLOR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Load Mask
        mask_bytes = np.fromfile(mask_path, dtype=np.uint8)
        mask_raw = cv2.imdecode(mask_bytes, cv2.IMREAD_UNCHANGED)
        
        # Unity exports masks sometimes as 3-channel (where R=G=B). 
        # Using [:,:,0] safely grabs the direct semantic class.
        if len(mask_raw.shape) == 3:
            mask = mask_raw[:,:,0]
        else:
            mask = mask_raw
            
        mask = mask.astype(np.int64) # PyTorch requirements for CrossEntropy

        # ======================================================================
        # SINIF EŞLEME SEÇENEKLERİ (CLASS MAPPING STRATEGIES)
        # ======================================================================
        """
        [Kullanıcı kararı bekleniyor, aşağıdakiler taslaktır]
        
        Seçenek 1 (4-Class Softmax - Varsayılan):
        Sınıflar (0,1,2,3) doğrudan beslenir. Model UNKNOWN pikselleri arka plan 
        olarak öğrenir. 
        # m_mapped = mask
        
        Seçenek 2 (Yol-Odaklı İkili Bölütleme - Binary Seg):
        Sadece "Yol" pikselini (1) = 1 yapar. (0,2,3) piksellerinin tamamını = 0 (Arkaplan) yapar.
        # m_mapped = (mask == 1).astype(np.int64)
        
        Seçenek 3 (Ignore Index Loss):
        Model sadece 1(Yol), 2(Engel), 3(Hedef) piksellerinden loss hesaplar. 
        0 (UNKNOWN) pikselleri loss'u etkilemez, maske kenarlarındaki belirsizlikler modele zarar vermez.
        Pytorch CrossEntropyLoss(ignore_index=0) ayarı gerektirir. Sınıflar (1,2,3) kalır.
        """

        if self.mapping_strategy == "Option2":
            mask = (mask == 1).astype(np.int64)

        if self.transform:
            img = self.transform(img)

        return img, torch.tensor(mask)

# Pytorch Dataloader Test 
# train_ds = DroneSegmentationDataset("dataset_split/train_pairs.csv")
# val_ds = DroneSegmentationDataset("dataset_split/val_pairs.csv")
# train_loader = DataLoader(train_ds, batch_size=8, shuffle=True)
