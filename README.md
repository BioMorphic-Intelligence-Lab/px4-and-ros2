# PX4 and ROS2
In this article, we want to provide a step-by-step tutorial on how to get PX4 working with ROS2 in hardware. 
We explain the process for a Pixhawk 6c and a RaspberryPi 4B but generally speaking, the same process should work for different hardware combinations with minor tweaks.

Let's get started!

## Prepare the Computers

The first step is to prepare the environment of each of the computers.

### Pixhawk 6c

The flight controller needs to be running the correct PX4 version. 
In particular, it needs to run a version that comes with the `microdds_agent` installed.

For this first, clone the [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) repository.
You can also choose which topics to communicate and which to keep quiet.
In [`PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml) you choose the messages you need and comment out the rest. 
Finally you make the right version via

    make px4_fmu-v6c_default

If you have a different flight computer make sure to choose the right version instead of v6c.
You can then flash the firmware onto the Pixhawk using a groundstation program, e.g. [QGroundControl](http://qgroundcontrol.com/).

Next, we need to set up some parameters that allow for communication over serial. 
I recommend setting the parameters directly over the PX4 shell as setting it over the QGroundControl interface can be buggy and result in wrong values depending on your version.

The parameters are documented on the [parameter reference](https://docs.px4.io/main/en/advanced_config/parameter_reference.html) site and the one we need for now is:

    param set UXRCE_DDS_CFG 102  # Turn on microdds_agent on startup and choose the right port (TELEM2)
    param set SER_TEL2_BAUD 921600  # Set baud rate 

To take effect a restart is required. 
We'll need to adjust some more parameters for various purposes but we'll get to that later.
Once restarted you can check if the `microdds_agent` is running by default by trying to start it again via

    uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600

Which should result in a message that the client is already running. 
In the above command `-t` defines the communication protocol (in this case `serial`), `-d` defines the device we're using to communicate (i.e. which port, where `/dev/ttyS3 == TELEM2` according to the [port mapping](https://docs.px4.io/main/en/flight_controller/pixhawk6c.html)), and `-d` defines the baud rate. 

### Raspberry Pi 4B
Make sure that the ubuntu version that is installed has all the newest dependencies:
    
    sudo apt-get update
    sudo apt-get upgrade

First, we need to enable serial communication on the RaspberryPi.
For this, we open the first install and then open the configuration menu via

    sudo apt-get install raspi-config
    sudo raspi-config

Navigate to InterfaceOptions with the arrow keys, and hit Enter.
Then, select SerialPort. 
Select No for the login shell and Yes for the serial port hardware.
Hit Enter on the final prompt, which will take you back to the main menu.
Navigate to the Finish button using the tab key, and hit Enter once more to exit.

If your device has built in bluetooth devices (e.g. the RaspberryPi 4), you also need to deactivate it.
For this add the following lines to the bottom of `/boot/firmware/config.txt`:

    enable_uart=1
    dtoverlay=disable-bt

, and reboot the computer via `sudo reboot`.

We created a Docker file that can be easily run on any ARM-based architecture that will take care of the communication on the RaspberryPi side. 
If you're interested in how we created the docker file, do read on, otherwise, feel free to skip this section and simply use the docker file as described in the software section. 

We use [Docker](https://www.docker.com/) to cross-compile an image for ARM-based systems on any other system.
Docker needs to be installed on the Pi via

    sudo apt-get install docker.io

For this, we define a docker file that specifies the recipe to create the desired image and then compile it. 
The `micro-ros-agent` docker file can be accessed [here](./docker/Dockerfile).

You can compile the docker file with the following command

    docker buildx build --platform linux/arm64,linux/amd64 --push -t docker_username/image_name

which will push the docker image to your docker repository. 
On the companion computer, you can then pull the docker image with

    docker pull docker_username/image_name

aka given the provided pre-compiled image:

    docker pull antbre/px4-uros-agent

_Note_: The `ros2` base image is only available for `arm64` and _not_ `arm` architectures. Therefore the companion computer needs to run a 64 Bit based arch. E.g. an Ubuntu 22.04 server or Raspbian x64!

After pulling this can be run with the following arguments:

    docker run --rm --net=host --privileged antbre/px4-uros-agent serial --dev /dev/ttyAMA0  -b 921600

Where `-it` opens it as an interactive container, `--privileged` gives it the required permissions, `--rm` is optional and indicates that the container is deleted after being stopped, `--net host` specifies the required network environment to let the ros2 communication work, and `-v` provides the required device permissions.  

## Physically connecting the Hardware

The next step is to physically connect the HW, i.e. the flight computer and the companion computer (in our case the Pixhawk 6c and the RaspberryPi 4B) both need to have access to power and need to be connected via a serial line. 

### Power

When flying you need to make sure the flight computer and the companion computer are both connected to a power source through independent voltage regulators. 
The pixhawk systems often include a regulator in shipping (such as [this one](https://holybro.com/collections/power-modules-pdbs/products/pm02-v3-12s-power-module)). 
For the RaspberryPi I chose to connect it via the power pins in the GPIO lineup through the onboard power distribution board and a programmable voltage regulator. 
Be aware that there are multiple ways to connect the RaspberryPi to a power source, e.g. via USB and this just happened to be my choice.  

For testing purposes, it is fine to simply hook up both the Pixhawk and the Pi via USB to some arbitrary power source, e.g. a power bank, your computer etc. 

### Serial Connection

To set up the serial connection you first need to solder the wire together. The key inside here is that we connect 
- RPi TX <-> RX Pixhawk
- RPi RX <-> TX Pixhawk

You can find a detailed description of how to wire things up [here](https://www.hackster.io/Matchstic/connecting-pixhawk-to-raspberry-pi-and-nvidia-jetson-b263a7).
Find a summary below:
- Take one of the 6-port cables delivered with the pixhawk and hold it with the red wire to the right.
- The cable wiring starts from the red wire and it reads as follows from the pixhawk side:

    |Pin | Signal | Voltage |
    |--- |  ---   |  ---    |
    |1(red) | VCC | +5 V |
    |2(black)|UART5_TX(out)|+3.3V|
    |3(black)|UART5_RX(in)|+3.3V|
    |4(black)|UART5_CTS(in)|+3.3V|
    |5(black)|UART5_RTS(out)|+3.3V|
    |6(black)|GND|GND|

- We only need the GND, the TX, and the RX wire, thus cut off all other cables at the base of one of the connectors.
- The above cables need to be fit to match the connection on the Pi, i.e. female jumpers need to be soldered to each end. 
- Once the soldering is done connect the wires as follows: 
    - Ground (black) to GPIO 9 (GND)
    - 6-pin TX (purple) to GPIO 10 (RX)
    - 6-pin RX (blue) to GPIO 8 (TX)
- Find the RaspberryPi pinout e.g. [here](https://pinout.xyz/).

## How to listen to and publish Messages

After doing all the above things you should be able to communicate between the RaspberryPi and the PX4. 
To check the functionality do a 
    
    ros2 topic list

on any computer that is in the network and see if the `/fmu/...` topics show up. 
Any ROS2 node that interacts on any of these topics needs to have the custom [PX4 ros messages](https://github.com/PX4/px4_msgs) sourced, i.e. the definitions need to be included in to ROS2 workspace and references as dependencies in the `CMakeList.txt` and `package.xml`. 

### Listening Messages from ROS2

Once the connection is established and the message definitions are available to a ROS2 node it can simply subscribe to the topics like any other ROS2 topic. 
No additional overhead is needed.

### Publishing Messages from ROS2

For publishing on these topics, generally speaking, the same is true. 
However, for the underlying modules (e.g. the EKF2 and mission controller) to use the respective messages some more overhead is needed. 

To operate in offboard mode the two types of messages must be published: 
 - messages of type [`OffboardControlMode.msg`](https://github.com/PX4/px4_msgs/blob/main/msg/OffboardControlMode.msg) that specify which type of reference to follow (Position, Attitude, Velocity, etc. ) must be published at a frequency $\geq 2 \text{Hz}$ to proof that the external controller is healthy
 - a reference, e.g. of type [`TrajectorySetpoint.msg`](https://github.com/PX4/px4_msgs/blob/main/msg/TrajectorySetpoint.msg) or [`VehicleAttitudeSetpoint.msg`](https://github.com/PX4/px4_msgs/blob/main/msg/VehicleAttitudeSetpoint.msg)

One tricky thing to get right is, that all of these messages have to have consistent timestamps with the px4 clock which (annoyingly) **is** not the ROS2 clock**. 
One way to get around this is to subscribe to the `/fmu/out/timesync_status` topic, which is of type [`TimesyncStatus.msg](https://github.com/PX4/px4_msgs/blob/main/msg/TimesyncStatus.msg), note the timestamp, manually count up from the last timestamp and publish the messages with the sum as the new timestamp. 

### Motion Capture 

To get the system running with MotionCapture the system needs to publish the position on the right topic with the right timestamp such that the internal observer uses the messages in its estimation. 

For this, we first need to set some parameters:

- Turn on EV aid (e.g. MoCap)
    - `EKF2_EV_CTRL = 13 == 1101_bin`
    - `EKF2_EVP_NOISE = 0.0100`
    - `EKF2_EVA_NOISE = 0.01`  
- Turn off Barometer
    - `EKF2_BARO_CTRL = 0`
- Turn off GPS
    - `GPS_1_CONFIG = 0`
    - `EKF2_GPS_CTRL = 0 == 0000_bin`
    - Be allowed to arm without
        - `COM_ARM_WO_GPS = 1`
- Turn off Magnetometer
    - `EKF2_MAG_TYPE  = 5`

We turn off GPS, Barometer and Magnetometer since they usually decrease the estimation performance indoors whereas the MotionCapture estimate is very good.

Next we need to publish the position reference on the `/fmu/in/vehicle_visual_odometry` topic of type [`VehicleOdometry.msg`](https://github.com/PX4/px4_msgs/blob/main/msg/VehicleOdometry.msg) (Note we're publishing on `/fmu/in/vehicle_visual_odometry` and **not** on `/fmu/in/vehicle_mocap_odometry` since the EKF2 observer internally only accesses the former). 
We use the same trick for the time stamps as above.
Also, note that the MoCap frequency cannot be too high, otherwise, you will overstrain the serial link and messages will get lost. 
The EKF2 will only use the odometry if it aligns with the current estimate.
I.e. the orientation needs to match and the position needs to be in the vicinity. 

As an example of how to publish the motion capture values from OptiTrack please refer to our [ROS2 node](https://github.com/BioMorphic-Intelligence-Lab/mocap_optitrack_client).

