import xml.etree.ElementTree as ET

# Constants
k_target = 2.6269644483e-5
rho = 1.2041
r_cp = 0.084
area = 0.002
a0 = 0.3

# Current k
cla_current = 4.2500
k_current = rho * (r_cp**2) * area * cla_current * a0

print(f"Current k: {k_current}")
print(f"Target k: {k_target}")

ratio = k_target / k_current
cla_new = cla_current * ratio

print(f"Required cla: {cla_new}")

# Test the force at 6874 RPM (719.8436 rad/s)
omega_max = 719.8436
force_max = k_target * (omega_max**2)
print(f"Max force at {omega_max} rad/s: {force_max} N")
print(f"In gram-force: {force_max / 0.00980665} gf")

# Modify SDF
tree = ET.parse('/home/furkan/s500_sim/ardupilot_gazebo/models/s500_custom_with_ardupilot/model.sdf')
root = tree.getroot()
model = root.find('model')

for plugin in model.findall('plugin'):
    if plugin.get('name') == 'ignition::gazebo::systems::LiftDrag':
        cla_elem = plugin.find('cla')
        if cla_elem is not None:
            cla_elem.text = f"{cla_new:.6f}"
    
    if plugin.get('name') == 'ArduPilotPlugin':
        for control in plugin.findall('control'):
            mult_elem = control.find('multiplier')
            if mult_elem is not None:
                val = float(mult_elem.text)
                if val > 0:
                    mult_elem.text = "719.8436"
                else:
                    mult_elem.text = "-719.8436"

tree.write('/home/furkan/s500_sim/ardupilot_gazebo/models/s500_custom_with_ardupilot/model.sdf', xml_declaration=True, encoding='utf-8')
