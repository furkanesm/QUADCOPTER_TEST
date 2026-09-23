import xml.etree.ElementTree as ET

tree = ET.parse('/home/furkan/s500_sim/ardupilot_gazebo/models/s500_custom_with_ardupilot/model.sdf')
root = tree.getroot()
model = root.find('model')

# Read calculated base_link values
with open('base_link_update.txt', 'r') as f:
    lines = f.readlines()
req_base_mass = lines[0].strip()
req_base_cg = lines[1].strip()
ixx = lines[2].strip()
ixy = lines[3].strip()
ixz = lines[4].strip()
iyy = lines[5].strip()
iyz = lines[6].strip()
izz = lines[7].strip()

# Update links
for link in model.findall('link'):
    name = link.get('name')
    inertial = link.find('inertial')
    if inertial is None: continue
    
    mass_elem = inertial.find('mass')
    mass = float(mass_elem.text)
    
    if name == 'base_link':
        mass_elem.text = req_base_mass
        
        i_pose = inertial.find('pose')
        if i_pose is None:
            i_pose = ET.SubElement(inertial, 'pose')
        i_pose.text = req_base_cg + " 0 0 0"
        
        inertia = inertial.find('inertia')
        inertia.find('ixx').text = ixx
        inertia.find('ixy').text = ixy
        inertia.find('ixz').text = ixz
        inertia.find('iyy').text = iyy
        inertia.find('iyz').text = iyz
        inertia.find('izz').text = izz
        
    elif 'rotor' in name and abs(mass - 0.025) < 1e-4:
        mass_elem.text = "0.010"
        inertia = inertial.find('inertia')
        for el in ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']:
            old_val = float(inertia.find(el).text)
            new_val = old_val * 0.4
            # Keep e-notation if it was there or format nicely
            inertia.find(el).text = f"{new_val:.10g}"

# Use a custom function to write without messing up namespaces or indentation too much
# But ElementTree might mess up the indentation of changed elements.
tree.write('/home/furkan/s500_sim/ardupilot_gazebo/models/s500_custom_with_ardupilot/model.sdf', xml_declaration=True, encoding='utf-8')
