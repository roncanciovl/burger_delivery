# ROS 2 Network Configuration

## Current Setup

Your ROS 2 installation is now configured for **network communication** between multiple PCs.

### Environment Variables

The following variables have been added to `~/.bashrc`:

```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### What Each Variable Does

1. **`ROS_DOMAIN_ID=42`**
   - Creates a "network group" for your ROS nodes
   - Only nodes with the **same domain ID** can communicate
   - Range: 0-101 (use different IDs for different robot systems)

2. **`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`**
   - `SUBNET` = Discover any node reachable via multicast (default).
   - `LOCALHOST` = Only discover nodes on the same machine.
   - `OFF` = Do not discover any other nodes.
   - *Note: This replaces the deprecated `ROS_LOCALHOST_ONLY` variable.*

3. **`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`**
   - Uses CycloneDDS middleware (better for multi-PC setups)
   - Alternative: `rmw_fastrtps_cpp` (default, but less reliable across networks)

## Network Information

- **WSL IP Address:** 172.27.119.126
- **Network:** Your local network (WiFi/Ethernet)

### Network Topology Diagram
![Network Diagram](ros_network_diagram.svg)

## 💻 WSL Networking Configuration (Windows Users)

WSL2 uses a virtual network (NAT) by default, which can hide ROS nodes from the rest of the physical network.

#### Option A: Mirrored Mode (Recommended & Easiest) ⭐
This mode makes WSL share the same IP address as your Windows host (`192.168.1.100`), making all ROS nodes visible instantly.

1. Open Windows Explorer and go to `%USERPROFILE%` (e.g., `C:\Users\YourUser`).
2. Create or edit a file named `.wslconfig`.
3. Add the following lines:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
4. Restart WSL from PowerShell:
   ```powershell
   wsl --shutdown
   ```
5. Verify in WSL: `ip addr` should now show your Windows IP.

#### Option B: Port Proxy (Alternative)
If you cannot use Mirrored mode, you must forward the DDS/micro-ROS ports from Windows to WSL.

Run in PowerShell as **Administrator**:
```powershell
# Forward micro-ROS Agent port
netsh interface portproxy add v4tov4 listenaddress=192.168.1.100 listenport=8888 connectaddress=<WSL_IP> connectport=8888
```
*(Note: You'll also need to forward DDS ports 7400-7500 for general ROS 2 discovery).*

## Configuring Other PCs

To connect another PC to this ROS 2 system:

### On Linux/Ubuntu PC:

1. Install ROS 2 Jazzy
2. Add to `~/.bashrc`:
   ```bash
   export ROS_DOMAIN_ID=42
   export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```
3. Restart terminal or run: `source ~/.bashrc`

### On Windows PC (with ROS 2):

1. Set environment variables (PowerShell as Admin):
   ```powershell
   [System.Environment]::SetEnvironmentVariable('ROS_DOMAIN_ID', '42', 'User')
   [System.Environment]::SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET', 'User')
   [System.Environment]::SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp', 'User')
   ```
2. Restart terminal

## Testing Network Communication

### Test 1: Verify Configuration

You can use the automated script for a complete diagnostic:
```bash
bash test_ros2_network.sh
```

**What the script verifies:**
1.  **Environment Variables:** Checks `ROS_DOMAIN_ID`, `ROS_AUTOMATIC_DISCOVERY_RANGE`, and `RMW_IMPLEMENTATION`.
2.  **Network Layer:** Extracts WSL IP and Gateway (Router) address.
3.  **Core ROS 2:** Verifies the `ros2` CLI installation and version.
4.  **Discovery Scan:** Lists visible nodes and topics (confirms discovery is working).
5.  **DDS Ports:** Checks if UDP ports 7400-7500 are active (the "engines" of ROS communication).
6.  **Local Pub Test:** Attempts to publish a test message to ensure internal ROS health.
7.  **Peer Guide:** Provides exact commands to test communication with a second PC.

Or run manual checks:
# In WSL
source ~/.bashrc
echo $ROS_DOMAIN_ID  # Should show: 42
echo $ROS_AUTOMATIC_DISCOVERY_RANGE  # Should show: SUBNET
```

### Test 2: Publish from PC1, Subscribe from PC2

**PC1 (this WSL):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub /test std_msgs/String "data: 'Hello from PC1'"
```

**PC2 (other computer):**
```bash
ros2 topic echo /test
# Should see: data: 'Hello from PC1'
```

### Test 3: See All Nodes on Network

```bash
ros2 node list  # Shows nodes from ALL PCs with domain ID 42
```

## Firewall Configuration

If nodes are not visible across PCs, check firewall:

### Windows Firewall (on Windows host):

```powershell
# Allow ROS 2 DDS ports (run as Admin)
New-NetFirewallRule -DisplayName "ROS2 Discovery (UDP)" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 7400-7500
New-NetFirewallRule -DisplayName "ROS2 Discovery (UDP)" -Direction Outbound -Action Allow -Protocol UDP -LocalPort 7400-7500
```

### Linux Firewall (on other PCs):

```bash
sudo ufw allow 7400:7500/udp
```

## Troubleshooting

### Nodes not visible across network?

1. **Check domain ID matches:**
   ```bash
   echo $ROS_DOMAIN_ID  # Must be same on all PCs
   ```

2. **Check discovery range setting:**
   ```bash
   echo $ROS_AUTOMATIC_DISCOVERY_RANGE  # Should be SUBNET (or unset defaults to SUBNET)
   ```

3. **Verify network connectivity:**
   ```bash
   ping <other-pc-ip>
   ```

4. **Check firewall** (see above)

5. **Restart ROS nodes** after changing environment variables

### Change Domain ID

To use a different domain (e.g., to isolate from other robots):

```bash
# Edit ~/.bashrc, change:
export ROS_DOMAIN_ID=99  # Use any number 0-101
```

Then restart terminal or `source ~/.bashrc`.

## Apply Changes

To apply the new configuration **immediately**:

```bash
source ~/.bashrc
```

Or close and reopen your terminal.

**Note:** Any ROS nodes already running will need to be restarted to use the new settings.
