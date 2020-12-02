# HIRO Skin Unit Setup
The skin unit shown below will provide you with IMU `/imu_data#` and proximity `/proximity_data#` data over wifi by publishing data to ROS topics. In this tutorial we will demonstrate how to connect a given skin unit to your WiFi and view live data in ROS. 

<img src="images/SU.jpg" alt="drawing" width="300"/>


## Hardware Setup
In order to complete all parts of this tutorial you should have access to: 

1. A sensor unit provided form the HIRO lab
1. [A USB to Micro connector](https://www.amazon.com/dp/B0711PVX6Z/ref=dp_prsubs_2)
1. [A battery](https://www.adafruit.com/product/258)


## Software Setup
We now outline the steps to make the sensor unit publish readings to a ROS topic over WiFi. 
1. [Download the Ardunio IDE](https://www.arduino.cc/en/software)
    * This will be used to upload code containing your network name and password
1. Clone this repository onto your computer with the command `git clone https://github.com/HIRO-group/skin_unit_setup.git`
1. Launch the Arduino IDE and open the file `YOUR_DOWNLOAD_LOCATION/skin_unit_setup/su_with_proximity/su_with_proximity.ino`, this can be done with `ctrl+O` or by selecting file->open
1. Connect the sensor unit to your computer with the USB to Micro connector so that code can be uploaded from the Arduino IDE
    * If you connect your battery to the sensor unit while it is plugged into your computer it will charge the battery
    * To show that the battery is charging a status LED will light up as seen below
    * At full battery the LED will turn off
    <img src="images/charging.jpg" alt="drawing" width="300"/>
1. In a terminal install [rosserial and rosserial-arduino](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup). If using ROS melodic run the following command or if you are using some other distro replace melodic with the output of `echo $ROS_DISTRO`:
    ```
    sudo apt-get install ros-melodic-rosserial-arduino
    sudo apt-get install ros-melodic-rosserial
    ```
   The rosserial package is used by the sensor unit to process data recived and parse it into a ros message. 

1. In the terminal utalize `rosserial-arduino` to create a `ros_lib` folder that Arduino will use to communicate with ROS. First navigate to the Ardunio libraries folder `cd YOUR_ARDUINO_LOCATION/libraries`, then run the following commands: 
    ```
    rm -rf ros_lib
    rosrun rosserial_arduino make_libraries.py .
    ```


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


