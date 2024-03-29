# PGV100 ROS2 Driver
This package is generated by @Ermanas and developed by @furkansariyildiz for ROS2. 

### PGV100 ROS1 Driver
```bash
https://github.com/Ermanas/pf_pgv100
```

### Setting the configuration parameters
```yaml
/pgv100_node:
  ros__parameters:
    pgv:
      port_name: "/dev/ttyACM0" # Port name of left motor controller serial port.
      baudrate: 115200 # Baudrate of the PGV100 serial port. 
      serial_port_group: "/dev" # Serial port group to detect tty
      id_vendor: "PGV" # ID Vendor to detect tty (PGV100 udevadm info)
      id_vendor_id: "20d2" # ID Vendor ID to detect tty (PGV100 udevadm info)
      id_model_id: "5740" # ID Model ID to detect tty (PGV100 udevadm info)
      id_path: "pci-0000:00:14.0-usb-0:4:1.0" # ID Path to detect tty (PGV100 udevadm info)
      timeout: 1000 # Timeout to open serial port (ms)
```

### To build pgv100-ros2 package
```
cd ~ros2_ws/src && git clone https://github.com/furkansariyildiz/pgv100-ros2.git
```

```bash
cd ~ros2_ws && colcon build --symlink-install --packages-select pgv100
```

### To run via ros2 run command
```bash
cd ~ros2_ws && source install/setup.bash
ros2 run pgv100 pgv-100
```

### To run via ros2 launch command (with config.yaml)
```bash
cd ~ros2_ws && source install/setup.bash
ros2 launch pgv100 pgv-100.launch.py
```

