# UR5 Pseudo Load


## Project Overview

This project aims to control a robotic system using real-time data to conduct tests on bone samples as part of orthopedic studies. The system consists of two main components:

- **Program on the Robot's  Teach pendant**: This program is responsible for executing the robot's movements based on received commands and processed data.
- **Control and Data Acquisition Program on PC**: Running on a computer, this program collects data from force-torque sensors and adjusts the robot's trajectory in real-time in response to detected forces.

The system enables the simulation of pseudo loads on the samples, based on user-defined inputs such as velocity vectors, force thresholds, and the number of cycles. The collected data are crucial for analyzing the mechanical responses of bones under various load conditions, which is vital for orthopedic research.


## System Requirements

- **Operating System**: The AMTI libraries used in this project are compatible with **Windows**. It is recommended to run this program on a Windows machine for full compatibility and functionality.

- **Virtual Machine**: While it's possible to run the program on a non-Windows system using a virtual machine (VM) configured with Windows, please note that this setup has not been tested. As such, we cannot guarantee full functionality or support for issues that may arise in a virtual machine environment.


## Installation

### Prerequisites

1. **Python 3.x**: Ensure Python is installed on your system.
2. **Libraries**: Required Python libraries include ctypes, matplotlib, numpy, os, sys, and logging. Install them using:
```sh
pip install matplotlib numpy ctypes os sys logging
```
3. **Hardware**: The transducer AMTI MC3A and its Gen5 amplifier, and a the UR5e robot.


### Setup Steps

You have two choices: 
- Use the USB drive AMTI force and Motion, go to file explorer and check what is the given name of the USB, let's say, for example, it is F:
```sh
F:
cd ur5-pseudo-load
```
- or clone this repository to your local directory. Let's say that you have chosen to put it in a USB drive, go to file explorer and check what is the given name of the USB, let's say, for example, it is G:
Press Windows + R, type "cmd", then "OK". In the terminal, type
```sh
G:
git clone https://github.com/Maintihami/ur5-pseudo-load.git
cd ur5-pseudo-load
```
2. Install the required libraries (see above).

## Project Directory Structure

The project directory is organized as follows to facilitate ease of navigation and clarity of purpose:

  - **Images/**: Contains images displayed in the readme 
  - **config/**: Contains configuration files, such as control loop parameters and system settings.
  - **docs/**: Documentation and product Manuals, software and drivers.
  - **lib/**: Library files
        - **rtde/**: Libraries and scripts related to Real-Time Data Exchange with the UR5 robot.
        - **Sensor/**: Libraries and drivers related to sensor data handling.

  - **output/**: Output files generated by the program, including logs, and data visualizations.
  - **Parts_Designs/**: contains the actual and futur proposed design files for individual mechanical components and assemblies needed for the project. 
        - Individual Part Files: Each file in Autodesk Inventor Part format (.ipt). These files are used directly for CNC machining or 3D printing. 
        - Assembly Files: Includes Autodesk Inventor Assembly files (.iam). These assemblies help in visualizing the end-product and are critical for ensuring that all parts align correctly before production.
  - **src/**: Source code for the main functionality of the project, including scripts for data collection and robot control.
  - **README**

Each directory is organized to maintain a clear separation of different aspects of the project, making it easier to manage and navigate.


## Usage

1. **Power On**: Ensure that the Teach pendant (tablette) is powered on.
Navigate to Open -> Program ->  new_folder_2 on the Teach pendant.
Select the 'start_point.urp' file click "open"
Go to the button "Power off" (buttom left corner), click on "ON", then on "START" ensure ALL green boxes are checked, then on "Exit".

2. **Mount the transducer**
- Ensure the power switch on the GEN5 signal conditioner is set to the off position. Plug the power supply into the Gen 5 and then into the power source.
- Attach the USB cable from the Gen 5 to the PC, then click "On" on the GEN5 signal conditioner.
If the sensor is not mounted;
  - Align the transducer face with the label, with the M8 connector.
  <img src="Images/montage.jpg" alt="Montage Image" width="400">
  - Mount the transducer with the tool flange.
- Attach the 7615 cable from the platform to the transducer input port.

3. **Start position**:
Start the program 'start_point.urp', by clicking on "play" (bottom right corner), then on "robot program"
You will need to press and hold the button "Move robot to :3: Waypoint_1", It will go to the initial position, Important; if you see that the robot is heading strait to bump into something, just press the free driver instead (the black button on the top back of the Teach pendant) while you are moving the robot with your hand to a safest position, then go back to press the button "Move robot to :3: Waypoint_1".
4. **AMTI-NetForce**: To see the forces and moments graphs in real time, Open AMTI-NetForce application, go to Amp ID press right array to select 1 on both windows, on the upper one go to Units, and set Fx, Fy to 1N per division and Fz to 5N per division, In the lower one, go to "Setup" set the Mx, My, Mz to 0.2N per division.
Go to Startup and select Hardware Zero, then select Start
<img src="Images/amti.jpg" alt="Montage Image" width="500">

4. **Mount the transducer with the spine**:
Press the free driver (the black button on the top back of the Teach pendant) while you are moving the robot with your hand to mount it with spine, make sure to align the screws with the sensor holes Screw in the bolts and fasten the nuts. 
Watch the graphs on the AMTI-NetForce, and Press the free driver to move the robot to a position where the forces and torques are almost 0,  Note that the Fz value cannot be fully zeroed.
<img src="Images/amti_graphs.png" alt="Force zeroed" width="500">
3. **Connect the robot to your PC**: Attach an ethernet cable to the Control box and to your computer.


6. **Set the payload**: On the Teach pendant go to 'Installation -> General -> Payload' and measure the payload, or just choose "loadcell" in the first drop-down bar if you are working with the actual set and spine.  
7. **Set the tool center position**: On the Teach pendant go to 'Installation -> General -> TCP', and set it up to the center of the intervertabrae, or just choose "loadcell_spine" in the first drop-down bar if you are working with the actual set and spine.
(for my sample it is z = 175mm)  

9. **Running the Program on PC**
Note the name of the directory
On the terminal, Navigate to the Source Directory: Change to the src directory where the Python scripts are located.  
```sh
cd src
```
**Running the classical spine tester**
This script will automatically program and execute a sequence of movements on the robot. The sequence includes:

- 5 cycles of flexion/extension: The robot performs five cycles and then returns to the initial position.
- 5 cycles of lateral bending: After completing the flexion/extension cycles, the robot executes five cycles of lateral bending and then returns to the initial position.
- 5 cycles of axial rotation: Finally, the robot performs five cycles of axial rotation before returning to the initial position.
The script also configures all necessary control parameters, allowing the robot to follow the predefined path seamlessly. User intervention is only required in case of an emergency stop.
To run this code, type:
```sh
python spine_tester.py
```  
**Running the general tester**
This script will ask the user for inputs, and you can execute all sorts of motions, having more flexibility.
To run this code, type:
```sh
python pseudo_load.py
```  
**Explanation of Inputs:**  
Enter the Necessary Inputs: During execution, the program will prompt you for specific inputs, such as the name of the force/torque output file, you should put .lvm as the extension, number of cycles for each movement, speed vectors and thresholds. Provide these inputs as required to ensure the robot operates under the correct parameters.  
- **Speed Vectors**: These determine the velocity at which the robot moves (m/s, m/s, m/s, rad/s, rad/s, rad/s). Ensure these values are set according to the required test parameters. You can freely choose the speed, but running it at 0.05m/s and 0.03rad/s is generally sufficient.  
Note that slower speeds gives higher precision
- **Force Thresholds**: These values set the limits for the forces applied during the tests. The robot will adjust its movements to stay within these thresholds.
Important: Don't forget the action-reaction rule. If you are working with a spine and you chose (0, 0, 0, 0.03, -0.03, 0) and you decided to monitor Mx and My, make sure to set the threshold at a value with the opposite sign (e.g., Mx= -4, My = 7).  

8. **Load the Program on Teach pendant**:
Navigate to Open -> Program -> new_folder_2 on the Teach pendant.
Select the `spine_testing.urp` file and click open to load it.
Select start then Then click 'play from beginning Robot Program' to start the robot program.  


11. **The end**: At the end of the program, you can stop the program running on the Teach pendant.  

**Stopping the Robot**:
- To safely stop the robot in case of an emergency, press the emergency stop button on the robot or on the  Teach pendant.  

## Demo Video

Here is a video demonstration of the project in action:
[Watch the Demo Video](https://youtu.be/MAbAeZJU6fM)


## Contributions and Collaboration

This repository is publicly accessible to share our work and findings with the community. However, as it is used in a controlled laboratory environment, it is important to maintain its integrity.


### How to Contribute

If you are interested in contributing or suggesting changes:
1. **Fork the Repository**: You can fork this repository to your own GitLab account.
2. **Make Your Changes**: Implement any changes or additions in your forked repository.
3. **Submit a Merge Request**: Propose your changes by submitting a merge request. Please provide a clear description of your changes and their purpose.

All contributions will be reviewed by the Orthopedic Biomechanics Research Laboratory (OBRL) at Colorado State University (CSU) or by the project lead. We value collaboration and suggestions, but please note that any modifications must be discussed and approved before being merged.

### Contact Information

- **Orthopedic Biomechanics Research Laboratory (OBRL), CSU**: [OBRL contact information or email]
- **Project Lead**: [Main Tihami Ouazzani] - [main.tihami2001@gmail.com]


## License

This project is open source and is licensed under the MIT License.


## Contact

For questions or suggestions, please contact [main.tihami2001@gmail.com]

