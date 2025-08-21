# 4WD SLAM Robot Simulation

This repository contains a complete ROS 2 simulation for a four-wheeled mobile robot, ready for navigation and SLAM tasks. The project uses Gazebo Fortress for realistic physics simulation and `ros2_control` for robust motor control. The robot is equipped with a LiDAR sensor and can be driven using the keyboard. (The navigation and SLAM is still on  progress)

## About The Project

This project provides a ready-to-use simulation environment for a four-wheel differential drive robot. It was built progressively, starting from a basic URDF model and culminating in a fully controllable Gazebo simulation.

### Key Features:
* **Simulation:** Utilizes Gazebo Fortress for high-fidelity physics and sensor simulation.
* **Standardized Control:** Employs the `ros2_control` framework with a `diff_drive_controller`.
* **LiDAR Equipped:** The robot model includes a GPU-accelerated LiDAR sensor for environment scanning, publishing data to the `/scan` topic.
* **Interactive Control:** Comes with a pre-configured launch file to drive the robot using the keyboard.

### Built With
* ROS 2 Humble
* Gazebo Fortress
* `ros2_control`

## File Structure

The repository is organized into two main packages:

* `four_wd_description`: Contains the robot's URDF model and sensor definitions.
* `four_wd_bringup`: Contains the launch files and configuration needed to run the simulation and controllers.

## Getting Started

Follow these steps to get the simulation up and running on your local machine.

### Prerequisites

You must have ROS 2 Humble and Gazebo Fortress installed. You will also need the `ros2_control` and Gazebo integration packages.
* **Install `ros2_control` and Gazebo plugins:**
    ```sh
    sudo apt update
    sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control ros-humble-ign-ros2-control
    ```
* **Install the keyboard teleop package:**
    ```sh
    sudo apt install ros-humble-teleop-twist-keyboard
    ```

### Installation & Building

1.  **Clone the repo into your ROS 2 workspace's `src` folder:**
    ```sh
    cd ~/ros2_ws/src
    git clone <your-repo-url>
    ```
2.  **Navigate to your workspace root, install dependencies, and build:**
    ```sh
    cd ~/ros2_ws
    rosdep install -i --from-path src -y
    colcon build --symlink-install
    ```

## Usage

To launch the simulation, run the main launch file from your workspace root.

1.  **Source your workspace:**
    ```sh
    source install/setup.bash
    ```
2.  **Run the simulation launch file:**
    ```sh
    ros2 launch four_wd_bringup simulation.launch.py
    ```
This command will:
* Start the Gazebo simulator with an empty world.
* Spawn your four-wheeled robot into the world.
* Launch all necessary `ros2_control` nodes and the `diff_drive_controller`.
* Open a **new terminal window** for keyboard control.

### Driving the Robot

1.  After launching, a new `xterm` window will appear.
2.  **Click on this new window** to give it focus.
3.  Use the following keys to drive the robot:
    * `i` - Move forward
    * `,` - Move backward
    * `j` - Turn left
    * `l` - Turn right
    * `k` - Stop
  
<img width="1907" height="1054" alt="Image" src="https://github.com/user-attachments/assets/200ae91e-2d34-4293-a987-da55d1152e12" />
