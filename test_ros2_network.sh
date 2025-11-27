#!/bin/bash
# ROS 2 Network Test Script
# This script verifies that ROS 2 can communicate across the network

echo "=========================================="
echo "ROS 2 Network Configuration Test"
echo "=========================================="
echo ""

# 1. Check ROS environment variables
echo "1. Checking ROS 2 Environment Variables:"
echo "   ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-NOT SET}"
echo "   ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-NOT SET}"
echo "   RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-default (FastRTPS)}"
echo ""

# 2. Check network connectivity
echo "2. Network Information:"
echo "   WSL IP Address: $(hostname -I | awk '{print $1}')"
echo "   Gateway: $(ip route | grep default | awk '{print $3}')"
echo ""

# 3. Test if ROS 2 is working
echo "3. Testing ROS 2 Installation:"
if command -v ros2 &> /dev/null; then
    echo "   ✓ ROS 2 command found"
    ros2 --version
else
    echo "   ✗ ROS 2 not found in PATH"
    exit 1
fi
echo ""

# 4. List active ROS 2 nodes
echo "4. Active ROS 2 Nodes:"
timeout 3 ros2 node list 2>/dev/null || echo "   No nodes detected (this is normal if nothing is running)"
echo ""

# 5. List active topics
echo "5. Active ROS 2 Topics:"
timeout 3 ros2 topic list 2>/dev/null || echo "   No topics detected"
echo ""

# 6. Network ports check
echo "6. Checking DDS Ports (7400-7500):"
if command -v netstat &> /dev/null; then
    netstat -tuln | grep -E '74[0-9]{2}' | head -5 || echo "   No DDS ports in use (normal if no ROS nodes running)"
else
    echo "   netstat not available"
fi
echo ""

# 7. Test publishing a message
echo "7. Testing Topic Publication (5 seconds):"
echo "   Publishing test message to /test_topic..."
timeout 5 ros2 topic pub /test_topic std_msgs/String "data: 'Network test from $(hostname)'" --once 2>/dev/null && echo "   ✓ Message published successfully" || echo "   ✗ Failed to publish"
echo ""

echo "=========================================="
echo "Test Complete!"
echo "=========================================="
echo ""
echo "To test communication with another PC:"
echo "1. On this PC: ros2 topic pub /hello std_msgs/String \"data: 'Hello from PC1'\""
echo "2. On other PC: ros2 topic echo /hello"
echo ""
echo "Both PCs must have ROS_DOMAIN_ID=0"
