import os

files = [
    'burger_description/urdf/burger_delivery_gen3.urdf',
    'burger_description/urdf/delivery_scene_fixed.urdf',
    'burger_description/vendor/kortex_description/robots/gen3_2f_adapter.urdf',
    'burger_description/vendor/robots/gen3_2f_adapter.urdf'
]

broken_text = """      <geometry>
        <cylinder radius="0.04" length="0.015"/>
      </geometry>
      <material name="adapter_black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
      </geometry>"""

fixed_text = """      <geometry>
        <cylinder radius="0.04" length="0.015"/>
      </geometry>
      <material name="adapter_black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>"""

for f in files:
    try:
        with open(f, 'r') as file:
            content = file.read()
            
        content = content.replace(broken_text, fixed_text)
        
        with open(f, 'w') as file:
            file.write(content)
        print(f"Fixed {f}")
    except Exception as e:
        print(f"Failed {f}: {e}")

# Also let's run check_urdf on one of them to be sure
os.system('check_urdf burger_description/urdf/delivery_scene_fixed.urdf')
