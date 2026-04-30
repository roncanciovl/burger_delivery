import re

files = [
    'burger_description/urdf/burger_delivery_gen3.urdf',
    'burger_description/urdf/delivery_scene_fixed.urdf',
    'burger_description/vendor/kortex_description/robots/gen3_2f_adapter.urdf',
    'burger_description/vendor/robots/gen3_2f_adapter.urdf'
]

# The mesh tag to replace is:
# <mesh filename="package://burger_description/vendor/2f_adapter_description/meshes/visual/2f_adapter/2f_adapter_base.dae"/>
# and collision:
# <mesh filename="package://burger_description/vendor/2f_adapter_description/meshes/collision/2f_adapter/2f_adapter_base.stl"/>

for f in files:
    try:
        with open(f, 'r') as file:
            content = file.read()
            
        # Replace visual mesh with cylinder
        content = re.sub(
            r'<mesh filename="package://burger_description/vendor/2f_adapter_description/meshes/visual/2f_adapter/2f_adapter_base\.dae"\s*/>',
            r'<cylinder radius="0.04" length="0.015"/>',
            content
        )
        
        # Replace collision mesh with cylinder
        content = re.sub(
            r'<mesh filename="package://burger_description/vendor/2f_adapter_description/meshes/collision/2f_adapter/2f_adapter_base\.stl"\s*/>',
            r'<cylinder radius="0.04" length="0.015"/>',
            content
        )
        
        # Add a dark grey material to the visual block of the adapter if not already there
        # Let's just wrap it in a material tag if we can, but URDF visual usually needs material.
        # Actually the URDF probably doesn't have a material on that specific link since dae files contain their own textures.
        # We need to add: <material name="dark_grey"><color rgba="0.2 0.2 0.2 1.0"/></material>
        # Let's find the visual block of gen3_2f_adapter_base_link and add the material right after geometry.
        
        with open(f, 'w') as file:
            file.write(content)
        print(f"Updated {f}")
    except Exception as e:
        print(f"Failed {f}: {e}")
