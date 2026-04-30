import re

files = [
    'burger_description/urdf/delivery_scene_fixed.urdf',
]

for f in files:
    try:
        with open(f, 'r') as file:
            content = file.read()
            
        content = re.sub(r'<link name="gen3_robotiq_85_(left|right)_[^"]+">.*?</link>', '', content, flags=re.DOTALL)
        content = re.sub(r'<joint name="gen3_robotiq_85_(left|right)_[^"]+" type="[^"]+">.*?</joint>', '', content, flags=re.DOTALL)
        content = content.replace('robotiq_85', '2f_adapter')
        content = content.replace('robotiq_description', '2f_adapter_description')
        content = content.replace('robotiq_base', '2f_adapter_base')
        
        with open(f, 'w') as file:
            file.write(content)
        print(f"Updated {f}")
    except Exception as e:
        print(f"Failed {f}: {e}")
