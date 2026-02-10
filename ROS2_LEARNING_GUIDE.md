# ROS2 Learning Guide: Using Part 1 as Teaching Project

## 📚 **Introduction to ROS2 Concepts**

This guide uses the **Part 1 Wheel Odometry project** as a hands-on teaching tool to learn ROS2 fundamentals. You'll understand core concepts through practical examples.

---

## 🔧 **1. ROS2 Nodes - The Building Blocks**

### **What is a Node?**
A **node** is a single-purpose executable that performs computation. Think of nodes as individual workers in a robot system.

### **In Part 1 Examples:**
```python
# From simple_wheel_odometry.py
class SimpleWheelOdometryNode(Node):
    def __init__(self):
        super().__init__('simple_wheel_odometry_node')  # Node name
        # This creates a ROS2 node named 'simple_wheel_odometry_node'
```

### **Key Node Concepts:**
- **Node Name**: Unique identifier (`simple_wheel_odometry_node`)
- **Single Purpose**: This node only computes wheel odometry
- **Independent**: Can run separately from other nodes
- **Communicates**: Through topics, services, and parameters

### **Practice Commands:**
```bash
# List all active nodes
ros2 node list

# Get info about specific node
ros2 node info /simple_wheel_odometry_node

# Run a node
ros2 run fra532_lab1_part1 simple_wheel_odometry
```

---

## 📡 **2. Topics - Data Highways**

### **What are Topics?**
**Topics** are named buses for message passing. Multiple nodes can publish/subscribe to the same topic.

### **In Part 1 Examples:**

#### **Subscriber (Input):**
```python
# Listening to wheel encoder data
self.joint_sub = self.create_subscription(
    JointState,           # Message type
    '/joint_states',      # Topic name  
    self.joint_states_callback,  # What to do with data
    10                    # Queue size
)
```

#### **Publisher (Output):**
```python
# Publishing odometry results
self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)

# Later in code:
odom_msg = Odometry()  # Create message
# ... fill message with data ...
self.odom_pub.publish(odom_msg)  # Send it out
```

### **Topic Flow in Part 1:**
```
Bag File → /joint_states → [Wheel Odom Node] → /wheel_odom → RViz2
          → /scan       →                    → /tf        → RViz2
```

### **Practice Commands:**
```bash
# List all topics
ros2 topic list

# See topic info
ros2 topic info /joint_states

# Monitor topic data
ros2 topic echo /wheel_odom

# Check message rate
ros2 topic hz /joint_states

# See topic structure
ros2 interface show sensor_msgs/msg/JointState
```

---

## 🎚️ **3. Parameters - Configuration Values**

### **What are Parameters?**
**Parameters** are configuration values that can be set at runtime without recompiling code.

### **In Part 1 Examples:**
```python
# Using parameters in node
self.wheel_radius = self.declare_parameter('wheel_radius', 0.033).value
self.wheel_separation = self.declare_parameter('wheel_separation', 0.160).value

# Or setting via launch file
parameters=[{'use_sim_time': use_sim_time}]
```

### **Parameter File (ekf_params.yaml):**
```yaml
ekf_odometry:
  ros__parameters:
    wheel_radius: 0.033
    wheel_separation: 0.160
    process_noise:
      position_x: 0.01
```

### **Practice Commands:**
```bash
# List node parameters
ros2 param list /simple_wheel_odometry_node

# Get parameter value
ros2 param get /simple_wheel_odometry_node use_sim_time

# Set parameter 
ros2 param set /simple_wheel_odometry_node wheel_radius 0.035

# Load parameters from file
ros2 param load /simple_wheel_odometry_node config/ekf_params.yaml
```

---

## 🗺️ **4. TF (Transform) System - Coordinate Frames**

### **What is TF?**
**TF** manages coordinate frame relationships. It answers: "Where is frame A relative to frame B?"

### **In Part 1 Examples:**

#### **Publishing Transforms:**
```python
from tf2_ros import TransformBroadcaster

# Create transform broadcaster
self.tf_broadcaster = TransformBroadcaster(self)

# Publish transform from odom to base_footprint
tf_msg = TransformStamped()
tf_msg.header.frame_id = 'odom'        # Parent frame
tf_msg.child_frame_id = 'base_footprint'  # Child frame
tf_msg.transform.translation.x = self.x   # Position
tf_msg.transform.rotation.w = cos(theta/2) # Orientation
self.tf_broadcaster.sendTransform(tf_msg)
```

#### **Static Transforms (in launch file):**
```python
# Robot geometry (never changes)
base_to_laser_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0.032', '0', '0.172', '0', '0', '0', 'base_link', 'base_scan']
    #          [x, y, z, roll, pitch, yaw, parent_frame, child_frame]
)
```

### **Part 1 TF Tree:**
```
map (RViz reference)
 └── odom (odometry frame)
     └── base_footprint (robot ground contact)  ← Published by wheel_odom
         └── base_link (robot center)          ← Static transform
             └── base_scan (laser scanner)     ← Static transform
```

### **Practice Commands:**
```bash
# View complete TF tree
ros2 run tf2_tools view_frames.py

# Monitor specific transform
ros2 run tf2_ros tf2_echo odom base_footprint

# List all transforms
ros2 topic list | grep tf
```

---

## 🚀 **5. Launch Files - Orchestration**

### **What are Launch Files?**
**Launch files** start multiple nodes and configure the system. Like a conductor for an orchestra.

### **In Part 1 Example (wheel_odom_with_rviz.launch.py):**
```python
def generate_launch_description():
    # Start wheel odometry node
    wheel_odometry_node = Node(
        package='fra532_lab1_part1',
        executable='simple_wheel_odometry',
        parameters=[{'use_sim_time': True}]
    )
    
    # Start static transforms
    map_to_odom_tf = Node(...)
    
    # Start RViz2
    rviz_node = Node(...)
    
    # Return all nodes to start
    return LaunchDescription([
        wheel_odometry_node,
        map_to_odom_tf, 
        rviz_node
    ])
```

### **Launch File Benefits:**
- **Coordination**: Start multiple nodes together
- **Configuration**: Set parameters for all nodes
- **Conditional**: Start nodes based on arguments
- **Reusable**: Same launch for different configurations

### **Practice Commands:**
```bash
# Run launch file
ros2 launch fra532_lab1_part1 wheel_odom_with_rviz.launch.py

# Run with arguments
ros2 launch fra532_lab1_part1 wheel_odom_with_rviz.launch.py use_sim_time:=false

# List available launch files
ros2 pkg executables fra532_lab1_part1
```

---

## 📦 **6. Messages - Data Structures**

### **What are Messages?**
**Messages** define the data structure for topic communication. Like envelopes with specific formats.

### **Part 1 Message Examples:**

#### **JointState (Input):**
```yaml
# sensor_msgs/msg/JointState
Header header
string[] name          # ["wheel_left_joint", "wheel_right_joint"]
float64[] position     # [left_angle, right_angle] 
float64[] velocity     # [left_vel, right_vel]
float64[] effort       # [left_torque, right_torque]
```

#### **Odometry (Output):**
```yaml
# nav_msgs/msg/Odometry  
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose      # Position + uncertainty
geometry_msgs/TwistWithCovariance twist    # Velocity + uncertainty
```

### **Practice Commands:**
```bash
# Show message structure
ros2 interface show sensor_msgs/msg/JointState
ros2 interface show nav_msgs/msg/Odometry

# See live message content
ros2 topic echo /joint_states --once
```

---

## ⚙️ **7. Quality of Service (QoS) - Message Reliability**

### **What is QoS?**
**QoS** policies control how messages are delivered (reliability, durability, etc.).

### **In Part 1 (qos_overrides.yaml):**
```yaml
/joint_states:
  reliability: reliable     # Guarantee delivery
  history: keep_last       # Only keep newest messages
  depth: 10               # Buffer size

/scan:
  reliability: best_effort  # Fast delivery, may lose messages
  durability: volatile     # Don't save for late joiners
```

### **Why QoS Matters:**
- **Sensors**: Often use `best_effort` (speed > reliability)
- **Commands**: Use `reliable` (must not lose)
- **Bag Playback**: Must match original QoS settings

---

## 🎯 **8. Practical Learning Exercises**

### **Exercise 1: Monitor Data Flow**
```bash
# Terminal 1: Start wheel odometry
ros2 launch fra532_lab1_part1 wheel_odom_with_rviz.launch.py

# Terminal 2: Play bag file  
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/ --clock --qos-profile-overrides-path qos_overrides.yaml

# Terminal 3: Watch the data
ros2 topic echo /joint_states    # Input data
ros2 topic echo /wheel_odom      # Output data
ros2 topic hz /joint_states      # Data rate
```

### **Exercise 2: Modify Parameters**
```bash
# Change wheel radius while running
ros2 param set /simple_wheel_odometry_node wheel_radius 0.040

# See effect in RViz2 (trajectory will change scale)
```

### **Exercise 3: Understand TF**
```bash
# Start system and generate TF tree
ros2 run tf2_tools view_frames.py

# Monitor robot pose in real-time
ros2 run tf2_ros tf2_echo map base_footprint
```

### **Exercise 4: Debug Issues**
```bash
# Check if all nodes are running
ros2 node list

# Verify topic connections
ros2 topic info /joint_states

# Check for TF problems  
ros2 run tf2_ros tf2_monitor
```

---

## 🔍 **9. Common ROS2 Debugging**

### **Node Issues:**
```bash
# Node not starting?
ros2 run fra532_lab1_part1 simple_wheel_odometry --ros-args --log-level DEBUG

# Node crashing?
ros2 launch fra532_lab1_part1 simple_wheel_odometry.launch.py --debug
```

### **Topic Issues:**  
```bash
# No data on topic?
ros2 topic list | grep joint_states
ros2 topic hz /joint_states

# Wrong message type?
ros2 topic info /joint_states
```

### **TF Issues:**
```bash
# Missing transforms?
ros2 run tf2_tools view_frames.py

# Transform timing issues?  
ros2 run tf2_ros tf2_monitor map base_footprint
```

---

## 🎓 **10. Advanced ROS2 Concepts (for later)**

### **Services vs Topics:**
- **Topics**: Continuous data streams (sensor readings)
- **Services**: Request-response communication (save map, reset odometry)

### **Actions:**
- Long-running tasks with feedback (navigate to goal)

### **Lifecycle Nodes:**
- Nodes with managed states (configure → activate → deactivate)

### **Component Architecture:**
- Multiple node classes in one process (performance optimization)

---

## 🚀 **Next Steps: Apply to Your EKF Implementation**

Now that you understand ROS2 fundamentals through Part 1:

1. **Implement EKF**: Fill in the TODO sections in `ekf_odometry.py`
2. **Compare Results**: Use both wheel and EKF odometry 
3. **Tune Parameters**: Modify `ekf_params.yaml`
4. **Analyze Performance**: Compare trajectories in RViz2

### **Key Learning Outcomes:**
- ✅ Understanding ROS2 node architecture
- ✅ Topic-based communication patterns  
- ✅ Parameter configuration and tuning
- ✅ TF coordinate frame management
- ✅ Launch file orchestration
- ✅ Message structures and QoS policies
- ✅ Debugging and monitoring tools

**Congratulations!** 🎉 You now understand ROS2 through practical implementation. Use these concepts to build more complex robotics systems!

---

## 📖 **Additional Resources**

- **ROS2 Documentation**: [docs.ros.org](https://docs.ros.org)
- **TF2 Tutorials**: `ros2 run tf2_tools view_frames.py` 
- **Message Interfaces**: `ros2 interface list`
- **Node Introspection**: `ros2 node info <node_name>`

**Happy Learning!** 🤖