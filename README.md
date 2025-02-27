# Robot Trajectory Saver
Saving and Visualising trajectory of robot made easy

## System Requirements
- **Operating System:** Ubuntu 22.04
- **ROS 2 Distribution:** Humble

## Dependencies
Ensure the following dependencies are installed before running the project:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Cloning the Repository
Clone this repository to your workspace:

```bash
mkdir ros2_ws
cd ros2_ws
mkdir src
cd src
git clone https://github.com/tirthvyas7/robot_trajectory_saver.git
```

## Building the Package:
Build and Source Packages with these commands
```bash
cd ../.. #Ensure you are in ros2_ws
colcon build
source install/setup.bash
```

## Running the Trajectory Saver:
To start the voice-controlled robot, run the following command:

```bash
ros2 launch bot_world saver.launch.xml
```
The above command will open gazebo classic and spawn the differential drive robot in a custom world and rviz2 along with the node which saves and publishes the trajectory, which can be seen in rviz2 as marker array.

## Saving Trajectory:
Open new terminal and source the workspace.
Then call the ```save_trajectory``` service to save the trajectory in a ```.yaml``` file.
```bash
ros2 service call save_trajectory amr_msgs/srv/Trajsave "{filename: 'trajectory.yaml', duration: 10}"
```
Here filename and duration(in secs) are the inputs and configurable.

## Running the Trajectory Visualiser:
To start the voice-controlled robot, run the following command:

```bash
ros2 launch bot_world robot.launch.xml
```
The above command will open gazebo classic and spawn the differential drive robot in a custom world and rviz2.

In new terminal, run the visualiser node which will read and publish the trajectory as marker array from the file.

```bash
ros2 run amr_nav visualiser
```

**Ensure you source the workspace and ROS2 Humble in every terminal you open.**


