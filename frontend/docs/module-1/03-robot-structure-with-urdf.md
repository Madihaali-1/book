---
sidebar_label: '3. Robot Structure with URDF'
---

# 3. Robot Structure with URDF

## Unified Robot Description Format (URDF)

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the kinematic and dynamic properties of a robot, including links, joints, and their relationships.

## Key Components

### Links
- Rigid bodies with physical properties
- Visual and collision models
- Inertial properties for dynamics simulation

### Joints
- Connections between links
- Joint types: revolute, prismatic, continuous, fixed, etc.
- Joint limits and dynamics

### Materials and Visuals
- Appearance properties for simulation and visualization
- Collision geometry for physics simulation

## URDF Structure

```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
</robot>
```

## Physical AI Applications

URDF models are essential for:
- Robot simulation in Gazebo
- Kinematic analysis and inverse kinematics
- Motion planning and control
- Visualization in RViz

## Xacro for Complex Models

Xacro (XML Macros) extends URDF with:
- Parameterization
- Inclusion of other files
- Mathematical expressions
- Macros for repeated structures

## Best Practices

- Use consistent naming conventions
- Organize complex models in multiple files
- Include proper inertial properties for simulation
- Validate URDF with `check_urdf` tool
- Use Xacro for parameterized and maintainable models

## Integration with ROS 2

URDF models are loaded into ROS 2 via:
- Robot State Publisher for TF transforms
- Joint State Publisher for joint states
- Controllers for joint actuation

## Next Steps

With the understanding of robot structure, we'll move to simulation frameworks in the next module, exploring Gazebo and Unity for digital twins.