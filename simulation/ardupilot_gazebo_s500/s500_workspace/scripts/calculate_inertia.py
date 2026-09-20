import numpy as np

def parallel_axis(I_cg, mass, d):
    # d is vector from reference point to CG (d = CG - ref)
    # Actually, if I_cg is around CG, the inertia around ref is:
    # I_ref = I_cg + mass * (dot(d,d)*I3 - outer(d,d))
    d = np.array(d)
    I3 = np.eye(3)
    return I_cg + mass * (np.dot(d, d) * I3 - np.outer(d, d))

def shift_inertia_to_cg(I_ref, mass, d):
    # I_cg = I_ref - mass * (dot(d,d)*I3 - outer(d,d))
    d = np.array(d)
    I3 = np.eye(3)
    return I_ref - mass * (np.dot(d, d) * I3 - np.outer(d, d))

# Target values
M_target = 2.4
CG_target = np.array([-0.00337, -0.00859, -0.10886])
I_target_CG = np.array([
    [ 0.01514745416, -0.00006385863, -0.00002517111],
    [-0.00006385863,  0.00978178314, -0.00093703508],
    [-0.00002517111, -0.00093703508,  0.01194201601]
])

# I_target_origin
I_target_origin = parallel_axis(I_target_CG, M_target, CG_target)

# Other links
links = [
    {"name": "odom", "mass": 0.15, "pos": [0, 0, -0.023], "I": np.diag([0.0001, 0.0002, 0.0002])},
    {"name": "imu", "mass": 0.15, "pos": [0, 0, -0.023], "I": np.diag([0.00001, 0.00002, 0.00002])},
    {"name": "rotor0", "mass": 0.025, "pos": [0.1718022, -0.1718022, 0], "I": np.diag([9.75e-06, 0.000166704, 0.000167604])},
    {"name": "rotor1", "mass": 0.025, "pos": [-0.1718022, 0.1718022, 0], "I": np.diag([9.75e-06, 0.000166704, 0.000167604])},
    {"name": "rotor2", "mass": 0.025, "pos": [0.1718022, 0.1718022, 0], "I": np.diag([9.75e-06, 0.000166704, 0.000167604])},
    {"name": "rotor3", "mass": 0.025, "pos": [-0.1718022, -0.1718022, 0], "I": np.diag([9.75e-06, 0.000166704, 0.000167604])},
]

M_other = 0.0
CG_other_sum = np.zeros(3)
I_other_origin = np.zeros((3,3))

for l in links:
    M_other += l["mass"]
    CG_other_sum += l["mass"] * np.array(l["pos"])
    I_other_origin += parallel_axis(l["I"], l["mass"], l["pos"])

# Calculate base_link requirements
M_base = M_target - M_other
CG_base = (M_target * CG_target - CG_other_sum) / M_base
I_base_origin = I_target_origin - I_other_origin
I_base_CG = shift_inertia_to_cg(I_base_origin, M_base, CG_base)

print(f"M_base: {M_base}")
print(f"CG_base relative to origin: {CG_base}")
# Note: base_link is at [0,0,-0.023] relative to origin. 
# So its inertial pose relative to base_link is CG_base - [0, 0, -0.023]
CG_base_link_frame = CG_base - np.array([0, 0, -0.023])
print(f"CG_base relative to base_link: {CG_base_link_frame}")
print(f"I_base_CG:")
print(I_base_CG)

eigenvalues = np.linalg.eigvalsh(I_base_CG)
print(f"Eigenvalues: {eigenvalues}")
if np.any(eigenvalues <= 0):
    print("WARNING: Non-positive definite inertia matrix!")
else:
    print("Inertia is positive definite.")

# Check triangle inequality
Ixx, Iyy, Izz = I_base_CG[0,0], I_base_CG[1,1], I_base_CG[2,2]
if (Ixx + Iyy < Izz) or (Ixx + Izz < Iyy) or (Iyy + Izz < Ixx):
    print("WARNING: Triangle inequality violated!")
else:
    print("Triangle inequality satisfied.")
