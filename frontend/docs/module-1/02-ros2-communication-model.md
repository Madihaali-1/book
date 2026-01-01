---
sidebar_label: '2. ROS 2 Communication Model'
---

# 2. ROS 2 Communication Model

## Communication Primitives

ROS 2 provides several communication mechanisms for physical AI systems:

### Topics (Publish/Subscribe)
- Asynchronous, one-to-many communication
- Ideal for sensor data streams and state updates
- Quality of Service (QoS) policies for reliability

### Services (Request/Response)
- Synchronous, one-to-one communication
- Suitable for configuration and control commands
- Request/response pattern with timeout handling

### Actions
- Asynchronous, goal-oriented communication
- With feedback and status updates
- Perfect for long-running tasks like navigation

## Quality of Service (QoS)

For physical AI systems, QoS profiles determine how messages are delivered:
- Reliability: Best effort vs. Reliable
- Durability: Volatile vs. Transient local
- History: Keep last N vs. Keep all
- Deadline and lifespan policies

## Node Communication

Nodes in ROS 2 communicate through:
1. Discovery phase using DDS
2. Creation of communication entities
3. Message exchange based on QoS policies
4. Lifecycle management for robust operation

## Best Practices for Physical AI

- Use appropriate QoS settings for real-time constraints
- Implement proper error handling and recovery
- Design for network resilience in multi-robot systems
- Consider security aspects for production deployment

## Implementation Example

```cpp
// Publisher example for sensor data
rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
  "laser_scan",
  rclcpp::QoS(10).reliable().durability_volatile()
);
```

## Next Steps

The next chapter will cover Robot Structure with URDF, explaining how to model robots in ROS 2.