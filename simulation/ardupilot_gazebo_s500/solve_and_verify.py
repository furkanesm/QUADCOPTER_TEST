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

def check_physically_valid(I, name):
    evals = np.linalg.eigvalsh(I)
    tol = 1e-8
    sym = np.allclose(I, I.T, atol=tol)
    pos_def = np.all(evals > -tol)
    ixx, iyy, izz = I[0,0], I[1,1], I[2,2]
    # Triangle inequality with some tolerance
    ti_x = iyy + izz - ixx > -tol
    ti_y = ixx + izz - iyy > -tol
    ti_z = ixx + iyy - izz > -tol
    valid = sym and pos_def and ti_x and ti_y and ti_z
    return valid, sym, evals, ti_x, ti_y, ti_z

# Load SDF
tree = ET.parse('/home/furkan/s500_sim/ardupilot_gazebo/models/s500_custom_with_ardupilot/model.sdf')
root = tree.getroot()
model = root.find('model')

links = []
for link in model.findall('link'):
    name = link.get('name')
    pose_elem = link.find('pose')
    # If the pose tag has relative_to, we throw error
    if pose_elem is not None and pose_elem.get('relative_to') is not None:
        raise ValueError(f"relative_to not supported in {name}")
    
    link_pos, link_rpy = parse_pose(pose_elem.text if pose_elem is not None else None)
    R_link = get_R(link_rpy)
    
    inertial = link.find('inertial')
    if inertial is None: continue
    
    mass = float(inertial.find('mass').text)
    
    i_pose = inertial.find('pose')
    cg_pos_local, cg_rpy_local = parse_pose(i_pose.text if i_pose is not None else None)
    
    # Check if we need to scale rotor mass
    if 'rotor' in name and abs(mass - 0.025) < 1e-4:
        mass = 0.010
        scale = 0.4
    else:
        scale = 1.0
        
    abs_cg = link_pos + R_link @ cg_pos_local
    R_total = R_link @ get_R(cg_rpy_local)
    
    inertia = inertial.find('inertia')
    I_local = np.array([
        [float(inertia.find('ixx').text), float(inertia.find('ixy').text), float(inertia.find('ixz').text)],
        [float(inertia.find('ixy').text), float(inertia.find('iyy').text), float(inertia.find('iyz').text)],
        [float(inertia.find('ixz').text), float(inertia.find('iyz').text), float(inertia.find('izz').text)]
    ]) * scale
    
    I_model = R_total @ I_local @ R_total.T
    
    links.append({
        'name': name,
        'mass': mass,
        'cg': abs_cg,
        'I_cg': I_model,
        'link_pos': link_pos,
        'R_link': R_link
    })

# Target values
target_mass = 2.550
target_cg = np.array([-0.00337, -0.00859, -0.10886])
target_I_cg = np.array([
    [ 0.01514745416, -0.00006385863, -0.00002517111 ],
    [-0.00006385863,  0.00978178314, -0.00093703508 ],
    [-0.00002517111, -0.00093703508,  0.01194201601 ]
]) * 1.0625

other_mass = 0
other_cg_sum = np.zeros(3)
other_I_origin = np.zeros((3,3))

base_link = None
for l in links:
    if l['name'] == 'base_link':
        base_link = l
        continue
    other_mass += l['mass']
    other_cg_sum += l['mass'] * l['cg']
    other_I_origin += parallel_axis(l['I_cg'], l['mass'], l['cg'])

target_I_origin = parallel_axis(target_I_cg, target_mass, target_cg)

req_base_mass = target_mass - other_mass
req_base_cg_model = (target_mass * target_cg - other_cg_sum) / req_base_mass
req_base_I_origin = target_I_origin - other_I_origin
req_base_I_cg_model = shift_inertia_to_cg(req_base_I_origin, req_base_mass, req_base_cg_model)

print(f"base_link Model CG: {req_base_cg_model}")

# Convert required CG and inertia to base_link local frame
# p_model = p_link + R_link * p_local => p_local = R_link^T * (p_model - p_link)
req_base_cg_local = base_link['R_link'].T @ (req_base_cg_model - base_link['link_pos'])
req_base_I_cg_local = base_link['R_link'].T @ req_base_I_cg_model @ base_link['R_link']

print(f"base_link Local CG: {req_base_cg_local}")
print(f"base_link Local Inertia:\n{req_base_I_cg_local}")
print(f"base_link Mass: {req_base_mass}")

valid, sym, evals, ti_x, ti_y, ti_z = check_physically_valid(req_base_I_cg_local, "base_link")
print(f"Symmetric: {sym}, Evals: {evals}")
print(f"Triangle Ineq (x,y,z): {ti_x}, {ti_y}, {ti_z}")

if not valid:
    print("STILL INVALID!")
else:
    print("VALID SOLUTION. Ready to apply.")

with open('base_link_update.txt', 'w') as f:
    f.write(f"{req_base_mass:.7f}\n")
    f.write(f"{req_base_cg_local[0]:.7f} {req_base_cg_local[1]:.7f} {req_base_cg_local[2]:.7f}\n")
    f.write(f"{req_base_I_cg_local[0,0]:.10f}\n")
    f.write(f"{req_base_I_cg_local[0,1]:.10f}\n")
    f.write(f"{req_base_I_cg_local[0,2]:.10f}\n")
    f.write(f"{req_base_I_cg_local[1,1]:.10f}\n")
    f.write(f"{req_base_I_cg_local[1,2]:.10f}\n")
    f.write(f"{req_base_I_cg_local[2,2]:.10f}\n")

