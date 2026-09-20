import re

file_path = "models/s500_custom_with_ardupilot/model.sdf"
with open(file_path, "r") as f:
    content = f.read()

# Tags to scale
tags = ["mass", "ixx", "ixy", "ixz", "iyy", "iyz", "izz"]

def replacer(match):
    tag = match.group(1)
    val_str = match.group(2)
    val = float(val_str)
    new_val = val * 1.0625
    
    # If the original value was in scientific notation, try to keep it, 
    # but regular format is also fine. Let's use %.10g to avoid trailing zeros
    # but have high precision.
    formatted = f"<{tag}>{new_val:.10g}</{tag}>"
    return formatted

for tag in tags:
    # Match <tag>value</tag>
    # Note: this relies on the tags being exactly <tag>val</tag> without attributes
    pattern = r"<(" + tag + r")>\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*</\1>"
    content = re.sub(pattern, replacer, content)

with open(file_path, "w") as f:
    f.write(content)

print("Scaling applied.")
