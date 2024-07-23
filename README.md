# AI Task 2

## Create a Catkin Workspace
1. Open a terminal and create a directory for your catkin workspace:
```
mkdir -p ~/catkin_ws/src
```

2. Go to the __'src'__ directory and initalize the workspace:
```
cd ~/catkin_ws/src
catkin_init_workspace
```

3. Go back to the root and build the workspace:
```
cd ~/catkin_ws
catkin_make
```

4. Source the setup file:
```
source devel/setup.bash
```

To automatically source the setup file each time you open a terminal, you can add the above command to your .bashrc file:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Clone Robot Arm Package from Smart Methods
1. Open a terminal and go to the __'src'__ directory of your workspace:
```
cd ~/catkin_ws/src
```

2. Clone the package repository using __'git clone'__:
```
git clone <repository_url>
```

3. Build the workspace and source the setup file:
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Install Dependencies 
For ROS Noetic, install the following dependencies:
```
sudo apt-get install ros-noetic-moveit
sudo apt-get install ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui
sudo apt-get install ros-noetic-gazebo-ros-control joint-state-publisher
sudo apt-get install ros-noetic-ros-controllers ros-noetic-ros-control
```

![Screenshot from 2024-07-23 21-39-51](https://github.com/user-attachments/assets/066c6d3a-f53b-4da6-b71e-79695f5fa5ca)

## Configure Arduino with ROS
1. Download the Arduino IDE:
Go to the [Arduino Software page](https://www.arduino.cc/en/software) and download the Arduino IDE 2.3.2 version for Linux (64-bit).

2. Extract the downloaded file.

3. Change to the extracted directory and run the install script:
```
cd arduino
ls
sudo ./install.sh
```

![Screenshot from 2024-07-15 00-51-45](https://github.com/user-attachments/assets/4a500b09-d7c3-4f80-985d-d5d62b2c5839)

4. Install dependencies:
```
sudo apt install ros-noetic-rosserial
sudo apt install ros-noetic-rosserial-arduino
sudo apt install ros-noetic-rosserial-python
```

5. Specify the folder for saving sketches and libraries:
Create a folder and tell arduino where it is by going to __File__ > __Preferences__ > Specify folder name in the __'Sketchbook Location'__ menu.

![image](https://github.com/user-attachments/assets/cac32be4-ecc6-42a0-922c-36b8907faeaf)

![image](https://github.com/user-attachments/assets/911b51ca-faca-41cc-80c6-21770a2e1967)

![image](https://github.com/user-attachments/assets/27fb1993-c4b3-42b2-abfd-167c1dbcf04d)

7. Install Rosserial Arduino Library:
Click on __Sketches__ > __Include Library__ > __Manage Libraries__. Then search for __'Rosserial Arduino Library'__ and click on __Install__.

![image](https://github.com/user-attachments/assets/5290e330-f61f-447a-91c5-998a016b7c6f)

![image](https://github.com/user-attachments/assets/2d4a5bb2-8a2a-4220-9451-6eaccda71202)

9. Test the library installation by running an example provided by the library:
Click on __File__ > __Examples__ > __Rosserial Arduino Library__ > __HelloWorld__

![image](https://github.com/user-attachments/assets/5f94afa1-8fec-4e09-bf2d-1d018d855cc0)

![image](https://github.com/user-attachments/assets/ad029c0b-5d0e-4e97-be1f-06c6bbda239b)

Then click on __Tools__ > __Board__ > __Arduino AVR Boards__ > __Arduino Uno__ > __Port__ > __/dev/ttyACM0 (Arduino Uno)__.

![image](https://github.com/user-attachments/assets/cb55663d-53a7-4f37-a184-b213fa14c2bb)

Then verify and upload the code and check what was uploaded by doing the following.

![Screenshot from 2024-07-17 07-40-59](https://github.com/user-attachments/assets/55338e35-6e2d-48ac-b53f-5440d39f5510)

To intialize ROS, open a  terminal and run:
```
roscore
```

![Screenshot from 2024-07-17 07-40-53 (1)](https://github.com/user-attachments/assets/e4cf4f6a-323f-40d0-a05d-130fd27f59ef)

To establish serial conncetion, open a second terminal and run:
```
sudo chmod 666 /dev/ttyACM0
```
This sets the proper permissions for our USB port.

In the same terminal, to run the serial client run:
```
rosrun rosserial_python serial_node.py/dev/ttyACM0
```
This will start the client, establish communication, and listen.

![Screenshot from 2024-07-17 07-40-48](https://github.com/user-attachments/assets/4cec9456-0314-4469-8917-1f53740a68bd)

To see what's being sent, open a terminal and run:
```
rostopic list
```
These are all the topics that are being currently run.

Then to see the messages being sent on the chatter topic, run:
```
rostopic echo /chatter
```
This is the string defined in our Arduino code.

![Screenshot from 2024-07-17 07-40-43](https://github.com/user-attachments/assets/f5e7474a-08a8-4338-8c4d-29e403e593d5)
