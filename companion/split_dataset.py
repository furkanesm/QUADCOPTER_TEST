import os
import json
import random
import csv

random.seed(42)

root_dir = r"D:\Unity\Drone Arazisi\DroneDatasetOutput\run_20260921_220520_2bc9f0"
out_dir = r"c:\Users\Murat Ege\OneDrive\Masaüstü\DroneTest\QUADCOPTER_TEST\companion\dataset_split"
os.makedirs(out_dir, exist_ok=True)

meta_dir = os.path.join(root_dir, "metadata")
img_dir = os.path.join(root_dir, "images", "train")
sem_dir = os.path.join(root_dir, "semantic")

print("Parsing metadata...")
groups = {} 
for meta_file in os.listdir(meta_dir):
    if not meta_file.endswith('.json'):
        continue
    
    frame_id = meta_file.replace('.json', '')
    path = os.path.join(meta_dir, meta_file)
    with open(path, 'r') as f:
        m = json.load(f)
        
    pos = m.get('cameraPosition', {})
    rot = m.get('cameraRotation', {})
    
    px, py, pz = round(pos.get('x',0), 2), round(pos.get('y',0), 2), round(pos.get('z',0), 2)
    rx, ry, rz, rw = round(rot.get('x',0), 2), round(rot.get('y',0), 2), round(rot.get('z',0), 2), round(rot.get('w',0), 2)
    alt = round(m.get('altitudeAboveGroundMeters', py), 2)
    
    pose_hash = f"P{px}_{py}_{pz}_R{rx}_{ry}_{rz}_{rw}"
    
    if pose_hash not in groups:
        groups[pose_hash] = {'alt': alt, 'frames': []}
    
    groups[pose_hash]['frames'].append(frame_id)

print(f"Found {len(groups)} distinct semantic/pose groups.")
if len(groups) != 27:
    print(f"WARNING: Expected 27 groups, found {len(groups)}")

altitudes = {}
for g_idx, (phash, data) in enumerate(groups.items()):
    data['id'] = f"G{g_idx:02d}"
    a = int(round(data['alt']))
    if a not in altitudes:
        altitudes[a] = []
    altitudes[a].append(phash)

print(f"Distinct altitudes found: {list(altitudes.keys())}")

# Sorting & Shuffling
sorted_alts = sorted(list(altitudes.keys()))
for a in sorted_alts:
    random.shuffle(altitudes[a])

val_splits = []
test_splits = []
train_splits = []

# First pass: try to put 1 group of each altitude into Val and Test
for a in sorted_alts:
    if len(altitudes[a]) > 0:
        val_splits.append(altitudes[a].pop(0))
    if len(altitudes[a]) > 0:
        test_splits.append(altitudes[a].pop(0))

# The rest goes to a remaining pool
remaining = []
for a in sorted_alts:
    remaining.extend(altitudes[a])
random.shuffle(remaining)

# Top up Val and Test to hit EXACTLY 4 each
while len(val_splits) < 4 and len(remaining) > 0:
    val_splits.append(remaining.pop(0))
while len(test_splits) < 4 and len(remaining) > 0:
    test_splits.append(remaining.pop(0))

# The rest belong to Train (~19)
train_splits.extend(remaining)

# Validate mututal exclusivity
s_train = set(train_splits)
s_val = set(val_splits)
s_test = set(test_splits)
assert len(s_train.intersection(s_val)) == 0, "Train and Val leak semantic bounds"
assert len(s_train.intersection(s_test)) == 0, "Train and Test leak semantic bounds"
assert len(s_val.intersection(s_test)) == 0, "Val and Test leak semantic bounds"

def get_stats(splits):
    total_frames = 0
    alts = {}
    frames_list = []
    for s in splits:
        alt = int(round(groups[s]['alt']))
        alts[alt] = alts.get(alt, 0) + 1
        total_frames += len(groups[s]['frames'])
        frames_list.extend(groups[s]['frames'])
    return {"total_groups": len(splits), "frames": total_frames, "altitudes_count": alts}, frames_list

train_s, f_tr = get_stats(train_splits)
val_s, f_v = get_stats(val_splits)
test_s, f_te = get_stats(test_splits)

# Check frame duplication overlaps
set_tr = set(f_tr)
set_v = set(f_v)
set_te = set(f_te)
assert len(set_tr.intersection(set_v)) == 0, "Exact RGB frames overlap between Train and Val"
assert len(set_tr.intersection(set_te)) == 0, "Exact RGB frames overlap between Train and Test"
assert len(set_v.intersection(set_te)) == 0, "Exact RGB frames overlap between Val and Test"

total_all = train_s['frames'] + val_s['frames'] + test_s['frames']
assert total_all == 4500, f"Expected 4500 frames, got {total_all}"

def save_csv(filename, split_hashes):
    f_path = os.path.join(out_dir, filename)
    with open(f_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["rgb_path", "semantic_path", "group_id", "pose"])
        for h in split_hashes:
            g = groups[h]
            for fr in g['frames']:
                rgb = os.path.join(img_dir, fr + ".png")
                if not os.path.exists(rgb):
                    rgb = os.path.join(img_dir, fr + ".jpg")
                sem = os.path.join(sem_dir, fr + ".png")
                writer.writerow([rgb, sem, g['id'], h])
    return f_path

p1 = save_csv("train_pairs.csv", train_splits)
p2 = save_csv("val_pairs.csv", val_splits)
p3 = save_csv("test_pairs.csv", test_splits)

# Determine altitude distributions success
missing_alts = False
for s in [train_s, val_s, test_s]:
    if len(s['altitudes_count']) < len(sorted_alts):
        missing_alts = True

summary = {
    "random_seed": 42,
    "limitations": "Bu ayrım aynı parkurun farklı kamera pozlarında pilot değerlendirmedir. Yeni parkur veya gerçek kamera başarısı ölçülmemektedir. H-yol bağlantısı kaynak etiketlerde eksiktir.",
    "validations": {
        "mutual_exclusivity": "Train, Val ve Test arasında ortak maske grubu (semantic pose) YOKTUR.",
        "exact_rgb_independence": "Kümeler arasında birebir aynı RGB karesi YOKTUR.",
        "frame_integrity": f"Toplam {total_all} çift eşleşti."
    },
    "stratification_success_status": "sağlanamadı" if missing_alts else "sağlandı",
    "stratification_note": "Üç yükseklik test/val gruplarına 4'erli kota yüzünden tam olarak eşit/kalıntısız parçalanamadı." if missing_alts else "Bütün kümelere (Train/Val/Test) her üç yükseklik varyasyonundan (pose) en az bir tane atanabildi.",
    "splits": {
        "train": train_s,
        "val": val_s,
        "test": test_s
    },
    "output_paths": {
        "train_csv": p1,
        "val_csv": p2,
        "test_csv": p3
    }
}

sum_path = os.path.join(out_dir, "split_summary.json")
with open(sum_path, 'w', encoding='utf-8') as f:
    json.dump(summary, f, indent=4, ensure_ascii=False)

print(json.dumps(summary, indent=2, ensure_ascii=False))
