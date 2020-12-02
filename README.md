# HIRO Skin Unit Setup
Documentation on how to setup the skin units from the HIRO group. The skin unit shown below will provide you with IMU `/imu_data#` and proximity `/proximity_data#` data over wifi by publishing data to ROS topics.

![](images/SU.jpg)





## Hardware Setup

## Software Setup


## Uploading New Code

## Visualization using PlotJuggler

Once you are running the skin unit ROS publishers with `rosrun rosserial_python serial_node.py tcp` (make sure to run `roscore` in another terminal), you can echo out the data with:

```sh
# to view imu data:
rostopic echo /imu_data1

# to view proximity sensor data:
rostopic echo /proximity_data1
```

However, these commands don't *visually* show what the sensor is sensing, so we highly recommend using [PlotJuggler](https://github.com/facontidavide/PlotJuggler) to visualize your data.

To install and use PlotJuggler:

```sh
sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros

# to run:
rosrun plot_juggler PlotJuggler
```

Upon running the above command, you'll be greeted with a cool meme, then a GUI that looks like this:


