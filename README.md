# DM_CAN-rocemo-driver
Driver for a can arm using damiao motors (like OpenArm, or the custom arm) compatible with Rocemo

## Dependencies
This driver uses the SocketCAN linux library and will only work on a linux computer. It was developped and tested on Ubuntu 24.04.
This driver depends on glaze, pinocchio, redis and eigen, similar to the main Rocemo program.

## Installation procedure
The driver needs to be built from the rocemo conda environment
```
conda activate rocemo
mkdir build
cd build
cmake .. && make -j8
```

## Usage
1. Create a config or check that there is a proper config file for the current robot setup. The config files are in the `config` folder. In particular, the `robot_model_file` needs to be provided in order for the driver to perform gravity compensation properly.

2. Enable the can interface on which the robot is connected with can fd.
```
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
```

3. To run the robot driver, use the following command:
```
./build/robot_redis_driver -c <path/to/config>
```

4. If a gripper is connected to the can chain, the `gripper_can_id` config parameter must be set to a non zero value, corresponding to the can id of the gripper. Otherwise, set it to zero.
