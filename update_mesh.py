import re

files = [
    'burger_description/urdf/burger_delivery_gen3.urdf',
    'burger_description/urdf/delivery_scene_fixed.urdf',
    'burger_description/vendor/kortex_description/robots/gen3_2f_adapter.urdf',
    'burger_description/vendor/robots/gen3_2f_adapter.urdf'
]

for f in files:
    try:
        with open(f, 'r') as file:
            content = file.read()
            
        # Replace the dae mesh with a cylinder
        content = re.sub(
            r'<mesh filename="package://burger_description/vendor/2f_adapter_description/meshes/visual/2f_85/2f_adapter_base.dae"\s*/>',
            r'<cylinder radius="0.04" length="0.015"/>\n      </geometry>\n      <material name="adapter_black">\n        <color rgba="0.1 0.1 0.1 1.0"/>\n      </material>',
            content
        )
        
        # Replace the stl collision mesh with a cylinder
        content = re.sub(
            r'<mesh filename="package://burger_description/vendor/2f_adapter_description/meshes/collision/2f_85/2f_adapter_base.stl"\s*/>',
            r'<cylinder radius="0.04" length="0.015"/>',
            content
        )
        
        with open(f, 'w') as file:
            file.write(content)
        print(f"Updated {f}")
    except Exception as e:
        print(f"Failed {f}: {e}")
