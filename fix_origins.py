import os

files = [
    'burger_description/urdf/burger_delivery_gen3.urdf',
    'burger_description/urdf/delivery_scene_fixed.urdf',
    'burger_description/vendor/kortex_description/robots/gen3_2f_adapter.urdf',
    'burger_description/vendor/robots/gen3_2f_adapter.urdf'
]

# Text to replace
old_visual = """    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.015"/>
      </geometry>
      <material name="adapter_black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>"""

new_visual = """    <visual>
      <origin xyz="0 0 0.0075" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.04" length="0.015"/>
      </geometry>
      <material name="adapter_black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>"""

old_collision = """    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.015"/>
      </geometry>
    </collision>"""

new_collision = """    <collision>
      <origin xyz="0 0 0.0075" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.04" length="0.015"/>
      </geometry>
    </collision>"""

for f in files:
    try:
        with open(f, 'r') as file:
            content = file.read()
            
        content = content.replace(old_visual, new_visual)
        content = content.replace(old_collision, new_collision)
        
        with open(f, 'w') as file:
            file.write(content)
        print(f"Fixed origins in {f}")
    except Exception as e:
        print(f"Failed {f}: {e}")

