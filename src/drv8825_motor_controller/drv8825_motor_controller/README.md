# DRV8825 Motor Controller ROS2 Node
This node is designed to interface with the Waveshare DRV8825 HAT for the Raspberry Pi (found [here](https://www.amazon.com/Stepper-Motor-HAT-Compatible-Microstepping/dp/B0B4MV9BCN/)). This HAT can drive up to two stepper motors using the DRV8825 chip and a couple of GPIO pins. It can take 8.2V to 28V power input and automatically converts it to 5V to power the Raspberry Pi and motor simultaneously. 

## Operation
### Pre-launch
If this is the first installation and run, navigate to the root directory of your ROS2 workspace (e.g. `home/thetis_calibration_ws/`) and execute:

```
colcon build --packages-select drv8825_motor_controller --symlink-install
```

Then, source the built package using:

```
source install/setup.bash
```

### Launching
To launch this node, executing the following command:
```
ros2 run drv8825_motor_controller drv8825
```
This will initialize a node using the default pins and settings defined in the table below. 
To change these settings, you can modify the `ros2 run` command like so:

```
ros2 run drv8825_motor_controller drv8825 --ros-args -p PARAMETER_NAME:=NEW_VALUE
```

Where `PARAMETER_NAME` is one of the parameter names listed in the table below, and `NEW_VALUE` is one of the valid values also in the table.

More information on ROS2 node arguments can be found [here](https://docs.ros.org/en/iron/How-To-Guides/Node-arguments.html)


Table 1: List of DRV8825 node parameters and their possible values
| Parameter Name | Default                  | Alt 1                    |
|----------------|--------------------------|--------------------------|
| `dir_pin`      | 13                       | 24                       |
| `step_pin`     | 19                       | 18                       |
| `enable_pin`   | 12                       | 4                        |
| `mode_pins`    | (16,17,20)               | (21,22,27)               |
| `step_mode`    | StepModes.SOFTWARE.value | StepModes.HARDWARE.value |

### Operation
This node operates by generating a PWM signal at 50% duty cycle to turn on and off the "step" pin of the DRV8825, thus stepping the motor.
The frequency of this signal changes the number of steps taken per second and therefore the angular velocity of the motor.

