import numpy as np

# Base link tensor computed in verify_inertia.py (the one we passed)
# Gereken base_link Atalet Matrisi (kendi CG'si etrafında):
# [[0.01068621, -0.00005239, 0.00009306],
#  [-0.00005239, 0.00498414, -0.00063568],
#  [0.00009306, -0.00063568, 0.00905884]]
# Wait, let's read the exact matrix from model.sdf instead.
import xml.etree.ElementTree as ET
tree = ET.parse('/home/furkan/s500_sim/ardupilot_gazebo/models/s500_custom_with_ardupilot/model.sdf')
for link in tree.getroot().find('model').findall('link'):
    if link.get('name') == 'base_link':
        ixx = float(link.find('inertial/inertia/ixx').text)
        iyy = float(link.find('inertial/inertia/iyy').text)
        izz = float(link.find('inertial/inertia/izz').text)
        ixy = float(link.find('inertial/inertia/ixy').text)
        ixz = float(link.find('inertial/inertia/ixz').text)
        iyz = float(link.find('inertial/inertia/iyz').text)
        break

I_base = np.array([
    [ixx, ixy, ixz],
    [ixy, iyy, iyz],
    [ixz, iyz, izz]
])

eigvals = np.linalg.eigvalsh(I_base)
eigvals = np.sort(eigvals)

print(f"Base link inertia tensor from model.sdf:\n{I_base}")
print(f"Principal moments (eigenvalues): {eigvals[0]:.10f}, {eigvals[1]:.10f}, {eigvals[2]:.10f}")
print(f"Triangle inequality check on principal moments:")
print(f"  {eigvals[0]:.10f} + {eigvals[1]:.10f} = {eigvals[0]+eigvals[1]:.10f}")
print(f"  Largest: {eigvals[2]:.10f}")
if eigvals[0] + eigvals[1] > eigvals[2]:
    print("  -> GEÇTİ (Sum of two smallest > largest)")
else:
    print("  -> KALDI (Physically invalid)")

