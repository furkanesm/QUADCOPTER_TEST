import os
import shutil
import csv
import json

base_dir = r"c:\Users\Murat Ege\OneDrive\Masaüstü\DroneTest\QUADCOPTER_TEST\companion"
split_dir = os.path.join(base_dir, "dataset_split")
pkg_dir = os.path.join(base_dir, "Kaggle_Road_Segmentation_Dataset")
zip_out = os.path.join(base_dir, "Kaggle_Road_Segmentation_Dataset.zip")

def convert_to_relative(abs_path):
    # D:\Unity\Drone Arazisi\DroneDatasetOutput\run_XXXX\images\train\frame_000000.png
    # Replace with images/train/frame_000000.png
    parts = abs_path.replace('\\', '/').split('/')
    if 'images' in parts:
        idx = parts.index('images')
        return "/".join(parts[idx:])
    elif 'semantic' in parts:
        idx = parts.index('semantic')
        return "/".join(parts[idx:])
    return os.path.basename(abs_path)

def build_pkg():
    if os.path.exists(pkg_dir):
        shutil.rmtree(pkg_dir)
    os.makedirs(pkg_dir)
    
    # Create target dirs
    os.makedirs(os.path.join(pkg_dir, "images", "train"))
    os.makedirs(os.path.join(pkg_dir, "semantic"))
    
    csv_files = ["train_pairs.csv", "val_pairs.csv", "test_pairs.csv"]
    
    copied_files = set()
    
    for c_file in csv_files:
        p = os.path.join(split_dir, c_file)
        if not os.path.exists(p):
            continue
            
        c_out = os.path.join(pkg_dir, c_file)
        with open(p, 'r', encoding='utf-8') as fin, open(c_out, 'w', encoding='utf-8', newline='') as fout:
            reader = csv.DictReader(fin)
            writer = csv.writer(fout)
            writer.writerow(["rgb_path", "semantic_path", "group_id", "pose"])
            
            for row in reader:
                old_rgb = row['rgb_path']
                old_sem = row['semantic_path']
                
                new_rgb_rel = convert_to_relative(old_rgb)
                new_sem_rel = convert_to_relative(old_sem)
                
                # Copy file physically if not already copied
                if new_rgb_rel not in copied_files:
                    dest = os.path.join(pkg_dir, new_rgb_rel.replace('/', os.sep))
                    if os.path.exists(old_rgb):
                        shutil.copy2(old_rgb, dest)
                        copied_files.add(new_rgb_rel)
                        
                if new_sem_rel not in copied_files:
                    dest = os.path.join(pkg_dir, new_sem_rel.replace('/', os.sep))
                    if os.path.exists(old_sem):
                        shutil.copy2(old_sem, dest)
                        copied_files.add(new_sem_rel)
                        
                writer.writerow([new_rgb_rel, new_sem_rel, row['group_id'], row['pose']])
                
    # Copy split_summary
    shutil.copy2(os.path.join(split_dir, "split_summary.json"), os.path.join(pkg_dir, "split_summary.json"))
    
    # Write README
    readme_content = """# Kaggle Road Segmentation Pilot Dataset

## EĞİTİM EŞLEMESİ DİKKAT
Unity kaynak maskesi orijinleri:
- 1 = Yol (FREE)
- 0 = UNKNOWN / etiketlenmemiş arka plan
- 2 = Engel (OBSTACLE)
- 3 = Hedef (TARGET)

**GridPlanner İç Sınıfları Kullanılmayacak:** GridPlanner FREE=0 kullanırken burada kaynak maskeler (1=Yol) geçerlidir. Eğitim sadece 1 (Yol) sınıfını 1 olarak çekip kalanı 0 yapacaktır.

**KUSUR UYARISI:** Kaynakta `0` (UNKNOWN) olan piksellerin tümü fiziksel engel veya araç geçemez alan değildir. Bilinen H hedefine giden gizli yol bağlantısı ve veri maskesindeki açıklıklar kaynakta eksiktir ve `0` etiketine sahiptir. Ancak `ignore_index` KULLANILMAYACAK; tüm 0,2,3 değerleri negatif (0) arka plana bastırılacaktır. Yalnızca mevcut doğrulanmış `1` pikselleri hedeftir.
"""
    with open(os.path.join(pkg_dir, "README.md"), 'w', encoding='utf-8') as f:
        f.write(readme_content)

    print("Copying done. Zipping...")
    shutil.make_archive(pkg_dir, 'zip', pkg_dir)
    print(f"Zip generated: {zip_out}")

if __name__ == "__main__":
    build_pkg()
