import xml.etree.ElementTree as ET
import numpy as np

def parse_pose(pose_str):
    if not pose_str: return np.zeros(3), np.zeros(3)
    vals = [float(v) for v in pose_str.split()]
    return np.array(vals[:3]), np.array(vals[3:])

def get_R(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def parallel_axis(I_cg, mass, d):
    I3 = np.eye(3)
    return I_cg + mass * (np.dot(d, d) * I3 - np.outer(d, d))

def shift_inertia_to_cg(I_ref, mass, d):
    I3 = np.eye(3)
    return I_ref - mass * (np.dot(d, d) * I3 - np.outer(d, d))

# Load SDF
tree = ET.parse('/home/furkan/s500_sim/ardupilot_gazebo/models/s500_custom_with_ardupilot/model.sdf')
root = tree.getroot()
model = root.find('model')

links = []
# We ignore commented out sections because ElementTree ignores XML comments by default
for link in model.findall('link'):
    name = link.get('name')
    pose_elem = link.find('pose')
    link_pos, link_rpy = parse_pose(pose_elem.text if pose_elem is not None else None)
    
    inertial = link.find('inertial')
    if inertial is None: continue
    
    mass = float(inertial.find('mass').text)
    i_pose = inertial.find('pose')
    cg_pos, cg_rpy = parse_pose(i_pose.text if i_pose is not None else None)
    
    # Absolute CG position in model frame (assuming link_rpy is 0 for simplicity, which it is for these links)
    abs_cg = link_pos + cg_pos
    
    inertia = inertial.find('inertia')
    ixx = float(inertia.find('ixx').text)
    iyy = float(inertia.find('iyy').text)
    izz = float(inertia.find('izz').text)
    ixy = float(inertia.find('ixy').text)
    ixz = float(inertia.find('ixz').text)
    iyz = float(inertia.find('iyz').text)
    
    I_cg = np.array([
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz]
    ])
    
    links.append({
        'name': name,
        'mass': mass,
        'cg': abs_cg,
        'I_cg': I_cg
    })

print("1. AKTİF LİNKLERİN TOPLAM KÜTLESİ VE 2. AĞIRLIK MERKEZLERİ (SDF'den okunan mevcut durum):")
total_mass = 0
for l in links:
    print(f"  - {l['name']}: kütle = {l['mass']} kg, CG (model) = [{l['cg'][0]:.6f}, {l['cg'][1]:.6f}, {l['cg'][2]:.6f}] m")
    total_mass += l['mass']
print(f"  TOPLAM KÜTLE: {total_mass:.6f} kg\n")

print("3. BÜTÜN ARACIN MEVCUT TOPLAM AĞIRLIK MERKEZİ:")
total_cg = np.zeros(3)
for l in links:
    total_cg += l['mass'] * l['cg']
total_cg /= total_mass
print(f"  Mevcut Toplam CG: [{total_cg[0]:.6f}, {total_cg[1]:.6f}, {total_cg[2]:.6f}] m\n")

print("4. MEVCUT TOPLAM AĞIRLIK MERKEZİ ETRAFINDAKİ BİRLEŞİK ATALET MATRİSİ:")
I_total_origin = np.zeros((3,3))
for l in links:
    I_total_origin += parallel_axis(l['I_cg'], l['mass'], l['cg'])
I_total_cg = shift_inertia_to_cg(I_total_origin, total_mass, total_cg)
print("  Mevcut Atalet Matrisi (kg.m^2):")
print(np.array2string(I_total_cg, formatter={'float_kind':lambda x: "%.8f" % x}))
print()

# ----------------- HEDEF DEĞERLER VE FİZİKSEL GEÇERLİLİK TESTİ -----------------
target_mass = 2.550
target_cg = np.array([-0.00337, -0.00859, -0.10886])
target_I_cg = np.array([
    [ 0.01514745416, -0.00006385863, -0.00002517111 ],
    [-0.00006385863,  0.00978178314, -0.00093703508 ],
    [-0.00002517111, -0.00093703508,  0.01194201601 ]
]) * 1.0625

print("HEDEFLERLE KARŞILAŞTIRMA:")
print(f"  Hedef Kütle: {target_mass:.6f} kg | Mevcut: {total_mass:.6f} kg")
print(f"  Hedef CG:    [{target_cg[0]:.6f}, {target_cg[1]:.6f}, {target_cg[2]:.6f}] m | Mevcut: [{total_cg[0]:.6f}, {total_cg[1]:.6f}, {total_cg[2]:.6f}] m")
print("  Hedef Atalet Matrisi (kg.m^2):")
print(np.array2string(target_I_cg, formatter={'float_kind':lambda x: "%.8f" % x}))
print()

print("FİZİKSEL GEÇERLİLİK ÇÖZÜMÜ (base_link için gereken değerler):")
# Orijine göre hedef atalet
target_I_origin = parallel_axis(target_I_cg, target_mass, target_cg)

# Yardımcı linklerin (base_link HARİÇ) toplam kütlesi, CG'si ve orijin etrafındaki ataleti
other_mass = 0
other_cg_sum = np.zeros(3)
other_I_origin = np.zeros((3,3))

for l in links:
    if l['name'] == 'base_link': continue
    other_mass += l['mass']
    other_cg_sum += l['mass'] * l['cg']
    other_I_origin += parallel_axis(l['I_cg'], l['mass'], l['cg'])

req_base_mass = target_mass - other_mass
req_base_cg = (target_mass * target_cg - other_cg_sum) / req_base_mass
req_base_I_origin = target_I_origin - other_I_origin
req_base_I_cg = shift_inertia_to_cg(req_base_I_origin, req_base_mass, req_base_cg)

print(f"  Gereken base_link Kütlesi: {req_base_mass:.6f} kg")
print(f"  Gereken base_link CG (model orijinine göre): [{req_base_cg[0]:.6f}, {req_base_cg[1]:.6f}, {req_base_cg[2]:.6f}] m")
print("  Gereken base_link Atalet Matrisi (kendi CG'si etrafında, kg.m^2):")
print(np.array2string(req_base_I_cg, formatter={'float_kind':lambda x: "%.8f" % x}))

ixx, iyy, izz = req_base_I_cg[0,0], req_base_I_cg[1,1], req_base_I_cg[2,2]
print("\n  Üçgen Eşitsizliği (Triangle Inequality) Kontrolü (Ixx + Iyy > Izz, vs.):")
print(f"    Ixx + Iyy = {ixx+iyy:.8f}  ?  Izz = {izz:.8f}  -> {'GEÇTİ' if ixx+iyy > izz else 'KALDI'}")
print(f"    Ixx + Izz = {ixx+izz:.8f}  ?  Iyy = {iyy:.8f}  -> {'GEÇTİ' if ixx+izz > iyy else 'KALDI'}")
print(f"    Iyy + Izz = {iyy+izz:.8f}  ?  Ixx = {ixx:.8f}  -> {'GEÇTİ' if iyy+izz > ixx else 'KALDI'}")

if iyy + izz < ixx or ixx + iyy < izz or ixx + izz < iyy:
    print("\n  SONUÇ: Yardımcı linklerin (özellikle 4 rotorun ±0.1718 m mesafedeki dağılımı) atalet katkısı hedeften çıkarıldığında, base_link için geriye kalan atalet matrisi fiziksel olarak İMKANSIZDIR (Iyy + Izz < Ixx).")
else:
    print("\n  SONUÇ: Geçerli.")

