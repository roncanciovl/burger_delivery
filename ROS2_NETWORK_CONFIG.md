# ROS 2 Network Configuration

## Current Setup

Your ROS 2 installation is now configured for **network communication** between multiple PCs.

### Environment Variables

The following variables have been added to `~/.bashrc`:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### What Each Variable Does

1. **`ROS_DOMAIN_ID=42`**
   - Creates a "network group" for your ROS nodes
   - Only nodes with the **same domain ID** can communicate
   - Range: 0-101 (use different IDs for different robot systems)

2. **`ROS_LOCALHOST_ONLY=0`**
   - `0` = Allow network discovery (nodes visible across PCs)
   - `1` = Local only (nodes only visible on same PC)

3. **`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`**
   - Uses CycloneDDS middleware (better for multi-PC setups)
   - Alternative: `rmw_fastrtps_cpp` (default, but less reliable across networks)

## Network Information

- **WSL IP Address:** 172.27.119.126
- **Network:** Your local network (WiFi/Ethernet)

## Configuring Other PCs

To connect another PC to this ROS 2 system:

### On Linux/Ubuntu PC:

1. Install ROS 2 Jazzy
2. Add to `~/.bashrc`:
   ```bash
   export ROS_DOMAIN_ID=42
   export ROS_LOCALHOST_ONLY=0
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```
3. Restart terminal or run: `source ~/.bashrc`

### On Windows PC (with ROS 2):

1. Set environment variables (PowerShell as Admin):
   ```powershell
   [System.Environment]::SetEnvironmentVariable('ROS_DOMAIN_ID', '42', 'User')
   [System.Environment]::SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0', 'User')
   [System.Environment]::SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp', 'User')
   ```
2. Restart terminal

## Testing Network Communication

### Test 1: Verify Configuration

```bash
# In WSL
source ~/.bashrc
echo $ROS_DOMAIN_ID  # Should show: 42
echo $ROS_LOCALHOST_ONLY  # Should show: 0
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
New-NetFirewallRule -DisplayName "ROS 2 DDS" -Direction Inbound -Protocol UDP -LocalPort 7400-7500 -Action Allow
New-NetFirewallRule -DisplayName "ROS 2 DDS" -Direction Outbound -Protocol UDP -LocalPort 7400-7500 -Action Allow
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

2. **Check localhost setting:**
   ```bash
   echo $ROS_LOCALHOST_ONLY  # Must be 0
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
