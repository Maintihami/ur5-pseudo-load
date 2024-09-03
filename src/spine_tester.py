import ctypes
import time
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import logging
from scipy.spatial.transform import Rotation as R

# This script initializes and sends the entire sequence of movements to the robot in one execution.
# It sets up the necessary control parameters, ensuring that the robot follows the pre-defined path
# without requiring further user intervention, except in cases of emergency stops or adjustments.

# The forces to monitor and their corresponding thresholds are specified in the base frame.
# Multiple forces can be monitored simultaneously, each with distinct thresholds and logic operators.
# The program allows the user to specify the speed vector in the base frame, which dictates the trajectory of the robot.

"""-----------------------------sensor initialization-----------------------------------"""
# Function to convert the force from the sensor frame to the base frame
def rotation(force):
    Tz = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, 1]
    
    ])
    forces = np.dot(Tz, force.T).T
    return forces

# Relative path starting from the directory where your Python script is located
relative_path = r"../lib/Sensor/AMTIUSBDevice - 64.dll"


# Construct the absolute path
dll_path = os.path.join(os.path.dirname(__file__), relative_path)

try:
    amti_dll = ctypes.CDLL(dll_path)
    print(f"Successfully loaded DLL: {dll_path}")
except OSError as e:
    print(f"Error loading DLL: {e}")
    exit(1)
'''Define the function prototypes as described in the DLL documentation. 
This tells Python how to call these functions and what type of data they
 return or expect as arguments.'''
# Initialize the DLL
amti_dll.fmDLLInit.restype = None

# Check if the device initialization is complete
amti_dll.fmDLLIsDeviceInitComplete.restype = ctypes.c_int

# Set data collection method to post messages
amti_dll.fmDLLPostDataReadyMessages.restype = None
amti_dll.fmDLLPostDataReadyMessages.argtypes = [ctypes.c_int]

# Start and stop data acquisition
amti_dll.fmBroadcastStart.restype = None
amti_dll.fmBroadcastStop.restype = None

# Get data (polling method)
amti_dll.fmDLLTransferFloatData.restype = ctypes.c_int
amti_dll.fmDLLTransferFloatData.argtypes = [ctypes.POINTER(ctypes.POINTER(ctypes.c_float))]

# Prototype pour définir les sensibilités DAC
amti_dll.fmSetDACSensitivityTable.restype = None
amti_dll.fmSetDACSensitivityTable.argtypes = [ctypes.POINTER(ctypes.c_float)]
# Prototype pour définir les offsets des canaux
amti_dll.fmSetChannelOffsetsTable.restype = None
amti_dll.fmSetChannelOffsetsTable.argtypes = [ctypes.POINTER(ctypes.c_float)]
# Initialiser la DLL
print("Initializing the DLL...")
amti_dll.fmDLLInit()

# Wait for initialization to complete
time.sleep(0.25)  # Sleep for 250 milliseconds
status = amti_dll.fmDLLIsDeviceInitComplete()
if status == 0:
    raise Exception("DLL has not completed initializing.")
elif status == 1:
    raise Exception("DLL initialized, but no signal conditioners are present.")
elif status == 2:
    print("DLL initialization complete and devices found.")


# define recommended settings
recommended_gains = (ctypes.c_long * 6)(8, 8, 8, 1, 1, 1)  # réglages de gains
recommended_excitations = (ctypes.c_long * 6)(2, 2, 2, 2, 2, 2)  #réglages d'excitations
recomended_dac_sensitivities = (ctypes.c_float * 6)(27.454, 27.930, 6.2936, 191.460, 191.4600, 146.300)
zero_offset = (ctypes.c_float * 6)(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)    #offsets
# Apply recommended settings
amti_dll.fmSetCurrentGains(recommended_gains)
amti_dll.fmSetCurrentExcitations(recommended_excitations)
amti_dll.fmSetDACSensitivityTable(recomended_dac_sensitivities)


# Create a dictionary to store the units based on the mode
units_dict = {
    0: ("N", "Nm", 'Configured data output mode to metric MSA 6 Compatible.'),  # Metric MSA 6 Compatible
    1: ("N", "Nm", 'Configured data output mode to metric fully conditioned.'),  # Metric Fully Conditioned
    2: ("lb", "lb-ft", 'Configured data output mode to english MSA 6 Compatible.'),  # English MSA 6 Compatible
    3: ("lb", "lb-ft", 'Configured data output mode to english Fully Conditioned.'),  # English Fully Conditioned
    4: ("bits", "bits", 'Configured data output mode to Bits MSA 6 Compatible.')  # Bits MSA 6 Compatible
}
# Function to get a valid mode from the user
def get_valid_mode():
    while True:
        try:
            mode = int(input("Enter the desired mode (0-4) [0&1: metric, 2&3: english, 4: Bits]: "))
            # mode =1
            if mode in units_dict:
                return mode
            else:
                print("Invalid input. Please enter a number between 0 and 4.")
        except ValueError:
            print("Invalid input. Please enter an integer between 0 and 4.")

# Configure data output mode
mode = 1
amti_dll.fmBroadcastRunMode(mode)
force_units, moment_units, commentaire = units_dict[mode]
print(commentaire)

# Function to get a valid signal frequency from the user
def get_valid_frequency(max_recommended=500):
    while True:
        try:
            frequency = int(input(f"Enter the desired signal frequency in Hz (recommended up to {max_recommended} Hz): "))
            if frequency > 0:
                if frequency > max_recommended:
                    print(f"Note: {frequency} Hz exceeds the recommended maximum of {max_recommended} Hz.")
                return frequency
            else:
                print("Invalid input. Please enter a positive integer.")
        except ValueError:
            print("Invalid input. Please enter an integer.")

# Set the acquisition rate
# signal_frequency = get_valid_frequency()
signal_frequency = 500
amti_dll.fmBroadcastAcquisitionRate(signal_frequency)       #The new acquisition rate will take affect with the next Start command
print(f"Acquisition rate set to {signal_frequency} Hz.")

# define the size of the packet
packet_size = 512       # 512 bytes means 128 float values with 4 bytes each
amti_dll.fmDLLSetUSBPacketSize(packet_size)
print(f"USB Packet size set to {packet_size} bytes")
# Define a pointer for data
data_pointer = ctypes.POINTER(ctypes.c_float)()
nbr_data_pooled = 128     # Number of float values to pool


# Listes pour stocker les données pour les forces et les moments
Fx_values, Fy_values, Fz_values = [], [], []
Mx_values, My_values, Mz_values = [], [], []
counter_values = []     #to store the counter received values, it does not include the none values
data_values = []        #to store the data received, it includes the none values

# Define the sample rate
# The  Nyquist–Shannon sampling theorem states that the sample rate must be at least twice the bandwidth of the signal to avoid aliasing

# sample_rate = 1/2*signal_frequency
sample_rate = 1/100

# Define the relative path for the output folder
output_folder = '../output'

# Get the absolute path to the script's directory
script_dir = os.path.dirname(__file__)

# Construct the full path to the output folder
output_path = os.path.join(script_dir, output_folder)

# Create the output folder if it doesn't exist
os.makedirs(output_path, exist_ok=True)

# Specify the relative path for the new output file
output_file_name = input("Enter the name of the output file.lvm : ")
output_file_path = os.path.join(output_path, output_file_name)
output_position_path = os.path.join(output_path, "position.txt")
"""-----------------------------control parameters-----------------------------------"""

# Add the parent directory of src to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../lib')))
# Add the rtde directory to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../lib/rtde')))

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import socket
"""this code stops the robot when the force limits are exceeded
the forces are read from the internal force torque sensor of the robot and compared to the thresholds"""
# Parameters for the RTDE communication
ROBOT_HOST = '192.168.1.50'     # IP adress of the robot 
ROBOT_PORT = 30004

# Define the relative path to the configuration file
config_folder = '../config'
config_file = 'control_loop_configuration_spine.xml'

# Construct the full path to the configuration file
config_filename = os.path.join(script_dir, config_folder, config_file)
keep_running = True
logging.getLogger().setLevel(logging.INFO)

# Charge the rdte configuration
conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe('state')
Vspeed_names, Vspeed_types = conf.get_recipe('Vspeed')
fsm_names, fsm_types = conf.get_recipe('FSM')
# Connexion to the robot
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# Get the controller version
con.get_controller_version()

# Send the recipes to the controller
con.send_output_setup(state_names, state_types)
Vspeed = con.send_input_setup(Vspeed_names, Vspeed_types)
fsm = con.send_input_setup(fsm_names, fsm_types)

Vspeed.input_double_register_0 = 0
Vspeed.input_double_register_1 = 0
Vspeed.input_double_register_2 = 0
Vspeed.input_double_register_3 = 0
Vspeed.input_double_register_4 = 0
Vspeed.input_double_register_5 = 0
fsm.input_int_register_0 = 0
fsm.input_int_register_1 = 0
# Fonctions utilitaires pour la conversion entre les listes et les points de consigne
def Vspeed_to_list(Vspeed):
    return [Vspeed.__dict__[f"input_double_register_{i}"] for i in range(6)]

def list_to_Vspeed(Vspeed, lst):
    for i in range(6):
        Vspeed.__dict__[f"input_double_register_{i}"] = lst[i]
    return Vspeed


# Function to check force limits based on user selection
def check_force_limits(selected_forces, thresholds, logic_op):
    def check_condition(force_data_lists):
        conditions = []
        exceeded_indices = []
        for i, force_list in enumerate(force_data_lists):
            threshold = thresholds[i]
            if threshold >= 0:  # Threshold is positive or zero
                exceeded = any(value >= threshold for value in force_list)
            else:  # Threshold is negative
                exceeded = any(value <= threshold for value in force_list)
                
            if exceeded:
                conditions.append(True)
                exceeded_indices.append(i)
            else:
                conditions.append(False)
        
        if logic_op == "or":
            return any(conditions), exceeded_indices
        elif logic_op == "and":
            return all(conditions), exceeded_indices
        else:
            raise ValueError("Invalid logic operator. Use 'or' or 'and'.")
    
    return check_condition

# Function to collect the initial inputs from the user
def collect_initial_inputs():
    Vspeed_list = ["0,0,0,0.03,0,0", "0,0,0,0,0.01,0", "0,0,0,0,0,0.03"]
    # Convert each string in Vspeed_list to a list of floats
    Vspeed_list = [[float(value.strip()) for value in Vspeed_input.split(',')] for Vspeed_input in Vspeed_list]
    thresholds_list = [[-6.0], [-6.0], [-6.0]]
    selected_forces_list = [[3], [4], [5]]
    logic_op_list = ["or", "or", "or"] 
    cycles_list = [5.0, 5.0, 5.0]
    return Vspeed_list, selected_forces_list, thresholds_list, logic_op_list, cycles_list



def convert_coordinates(O):
    """
    Convert the coordinates of point O to a new point A using a fixed translation vector.
    
    :param O: List of 6 elements where the first 3 are the cartesian coordinates and the last 3 are the angles (in radians).
    :return: List of 6 elements representing the new coordinates of point A.
    """
    # Decompose the input vector
    O_cartesian = O[:3]  # First 3 elements are the cartesian coordinates
    angles = O[3:]  # Last 3 elements are the angles in radians

    # Define the fixed translation vector P' = (7.7, 7.7, 15.8)
    translation = [7.7, 7.7, 15.8]

    # Create a rotation matrix from the Euler angles (XYZ)
    rotation_matrix = R.from_euler('xyz', angles).as_matrix()

    # Translate point O by the translation vector
    O_prime = np.array(O_cartesian) - np.array(translation)

    # Apply the rotation to O_prime
    O_rotated = rotation_matrix.dot(O_prime)

    # Translate again to get the coordinates of A
    A_cartesian = O_rotated + np.array(translation)

    # The angles remain the same after translation
    A = np.concatenate([A_cartesian, angles])

    return A.tolist()


# Collect all inputs at the start
Vspeed_list, selected_forces_list, thresholds_list, logic_op_list, cycles_list = collect_initial_inputs()
# Initialize the first set of inputs
current_index = 0
current_cycle = 0
Vspeed_input = Vspeed_list[current_index]
selected_forces = selected_forces_list[current_index]
thresholds = thresholds_list[current_index]
logic_ops= logic_op_list[current_index]
check_limits = check_force_limits(selected_forces, thresholds, logic_ops)
Vspeed1 = list_to_Vspeed(Vspeed, Vspeed_input)
selected_cycle = cycles_list[current_index]
con.send(Vspeed1)

# Démarrer la synchronisation des données
if not con.send_start():
    sys.exit()



# Writing the header to the file (only once at the beginning)
with open(output_file_path, 'w') as f:
    f.write("LabVIEW Measurement\n")
    f.write("Writer_Version\t2\n")
    f.write("Reader_Version\t2\n")
    f.write("Separator\tTab\n")
    f.write("Decimal_Separator\t.\n")
    f.write("Multi_Headings\tNo\n")
    f.write("X_Columns\tOne\n")
    f.write("Time_Pref\tAbsolute\n")
    f.write("Operator\tPythonScript\n")
    f.write(f"Date\t{time.strftime('%Y/%m/%d')}\n")
    f.write(f"Time\t{time.strftime('%H:%M:%S')}\n")
    f.write("***End_of_Header***\n")
    f.write("\n")
    f.write("Channels\t6\n")
    f.write("Samples\t{0}\n".format(nbr_data_pooled // 8))
    f.write(f"Y_Unit_Label\t{force_units}\t{force_units}\t{force_units}\t{moment_units}\t{moment_units}\t{moment_units}\n")
    f.write("X_Dimension\tTime\tTime\tTime\tTime\tTime\tTime\n")
    f.write("X0\t0.0000000000000000E+0\n")
    f.write(f"Delta_X\t{sample_rate}\n")

    #
    f.write(f"packet size: {packet_size}bytes\n")
    f.write(f"number of packets: {nbr_data_pooled/8}\n")
    f.write(f"signal frequency: {signal_frequency}\n")
    f.write(f"sample frequency: {1/sample_rate}\n")
    #
    f.write("***End_of_Header***\n")


amti_dll.fmBroadcastResetSoftware()     #Apply the settings
time.sleep(0.5)  # Sleep for at least 250 milliseconds, Wait for the settings to take effect
# Zero the platform
amti_dll.fmBroadcastZero()
time.sleep(0.5)  # Wait for the zero command to take effect
print("Platform zeroed.")

# Start data acquisition
print("Starting data acquisition...")
amti_dll.fmBroadcastStart()
time.sleep(0.91)  # Wait for the start command to take effect

print("Data acquisition started.")
# Fonctions utilitaires pour la conversion entre les listes et les points de consigne
def Vspeed_to_list(Vspeed):
    return [Vspeed.__dict__[f"input_double_register_{i}"] for i in range(6)]

def list_to_Vspeed(Vspeed, lst):
    for i in range(6):
        Vspeed.__dict__[f"input_double_register_{i}"] = lst[i]
    return Vspeed

loop_frequencies = []
t0 = time.time()
Fx, Fy, Fz = [], [], []
Mx, My, Mz = [], [], []
tcp_pose_list = []
"""-----------------------------main loop-----------------------------------"""
try:
    print("Collecting data. Press Ctrl+C to stop.")
    while True:
        try:
            
            loop_t0 = time.time()
            result = amti_dll.fmDLLTransferFloatData(ctypes.byref(data_pointer))
            if result == 1:
                data = ctypes.cast(data_pointer, ctypes.POINTER(ctypes.c_float *nbr_data_pooled)).contents

                data_values.append([data[i] for i in range(nbr_data_pooled) if i % 8 == 0])  # Store data, here using the first element for simplicity
                # Extract the individual components in the transducer frame
                counter = [data[i] for i in range(nbr_data_pooled) if i % 8 == 0]
                Fx = [data[i] for i in range(nbr_data_pooled) if i % 8 == 1]
                Fy = [data[i] for i in range(nbr_data_pooled) if i % 8 == 2]
                Fz = [data[i] for i in range(nbr_data_pooled) if i % 8 == 3]
                Mx = [data[i] for i in range(nbr_data_pooled) if i % 8 == 4]
                My = [data[i] for i in range(nbr_data_pooled) if i % 8 == 5]
                Mz = [data[i] for i in range(nbr_data_pooled) if i % 8 == 6]
                # Combine force components into an array of vectors
                forces = np.array(list(zip(Fx, Fy, Fz)))
                torques = np.array(list(zip(Mx, My, Mz)))
                # Apply rotation transformation to force vectors
                rotated_forces = rotation(forces)
                rotated_moments = rotation(torques)
                # Extract the individual components
                # the force and moment values are in the tool frame
                Fx = rotated_forces[:, 0].tolist()
                Fy = rotated_forces[:, 1].tolist()
                Fz = rotated_forces[:, 2].tolist()
                Mx = rotated_moments[:, 0].tolist()
                My = rotated_moments[:, 1].tolist()
                Mz = rotated_moments[:, 2].tolist()

                # Write into the .lvm file
                with open(output_file_path, 'a') as f:
                    for i in range(len(counter)):
                        f.write(f"{counter[i]:<10}\t")
                        f.write(f"{Fx[i]:<10.5f}\t")
                        f.write(f"{Fy[i]:<10.5f}\t")
                        f.write(f"{Fz[i]:<10.5f}\t")
                        f.write(f"{Mx[i]:<10.5f}\t")
                        f.write(f"{My[i]:<10.5f}\t")
                        f.write(f"{Mz[i]:<10.5f}\n")

                # Add the data to the lists
                counter_values.extend(counter)
                Fx_values.extend(Fx)
                Fy_values.extend(Fy)
                Fz_values.extend(Fz)
                Mx_values.extend(Mx)
                My_values.extend(My)
                Mz_values.extend(Mz)


            else:
                # print("No new data available")
                data_values.append(None)  # Add a placeholder for no data

            time.sleep(sample_rate)  # Adjust the sleep time as necessary
            loop_t1= time.time()
            loop_duration = loop_t1 - loop_t0
            loop_frequency = 1 / loop_duration # Calculate the loop frequency
            loop_frequencies.append(loop_frequency)
            

            # """-------------------------------control loop--------------------------------"""
            state = con.receive()
            if state is None:
                break
            # print(f"Received state: {state}")
            fsm.input_int_register_0  = 1
            con.send(fsm)
            # print(state.actual_TCP_pose)
            tcp=state.actual_TCP_pose
            tcp_pose_list.append(tcp)
            if state.output_int_register_0 != 0:
                con.send(Vspeed1)
                

            # Check force limits
            force_data_lists = [Fx, Fy, Fz, Mx, My, Mz]
            limits_exceeded, exceeded_indices = check_limits([force_data_lists[i] for i in selected_forces])
            if limits_exceeded:
                print("Force limits exceeded.")
                with open(output_file_path, 'a') as f:
                    f.write(f"Force limits exceeded at cycle {current_cycle}.\n")
                fsm.input_int_register_0 = 0
                con.send(fsm)
                
                force_names = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
                for index in exceeded_indices:
                    if thresholds[index] >= 0:
                        print(f"{force_names[selected_forces[index]]} has exceeded its threshold with a value of {max(force_data_lists[selected_forces[index]])} ({force_units}/{moment_units})")
                    else:
                        print(f"{force_names[selected_forces[index]]} has exceeded its threshold with a value of {min(force_data_lists[selected_forces[index]])} ({force_units}/{moment_units})")
                    continue
                time.sleep(2)               # Wait for the robot to stop
                current_cycle += 0.5        # Increment the cycle count by 0.5
                if current_cycle < cycles_list[current_index]-0.5:
                    #initialize the force and moment values
                    Fx, Fy, Fz = [], [], []
                    Mx, My, Mz = [], [], []
                    force_data_lists = [Fx, Fy, Fz, Mx, My, Mz]
                    Vspeed_input = [-v for v in Vspeed_input]
                    thresholds = [-t for t in thresholds]
                    Vspeed1 = list_to_Vspeed(Vspeed, Vspeed_input)
                    con.send(Vspeed1)
                    check_limits = check_force_limits(selected_forces, thresholds, logic_ops)
                    
                    fsm.input_int_register_0 = 1
                    con.send(fsm)
                else:
                    # move to initial position
                    fsm.input_int_register_1 = 1
                    con.send(fsm)
                    print("All cycles have been completed. Moving to the initial position.")
                    while state.output_int_register_0 != 2:
                        state = con.receive()
                    fsm.input_int_register_1 = 0
                    con.send(fsm)
                    # Move to the next set of inputs
                    current_index += 1
                    current_cycle = 0
                    #initialize the force and moment values
                    Fx, Fy, Fz = [], [], []
                    Mx, My, Mz = [], [], []
                    force_data_lists = [Fx, Fy, Fz, Mx, My, Mz]
                    if current_index < len(Vspeed_list):
                        Vspeed_input = Vspeed_list[current_index]
                        selected_forces = selected_forces_list[current_index]
                        thresholds = thresholds_list[current_index]
                        logic_ops = logic_op_list[current_index]
                        check_limits = check_force_limits(selected_forces, thresholds, logic_ops)
                        Vspeed1 = list_to_Vspeed(Vspeed, Vspeed_input)
                        con.send(Vspeed1)
                        # fsm.input_int_register_0 = 1
                        # fsm.input_int_register_1 = 0
                        # con.send(fsm)
                        
                    else:
                        print("All input sets have been used. Stopping.")
                        fsm.input_int_register_0 = 0
                        fsm.input_int_register_1 = 1
                        con.send(fsm)
                        time.sleep(2)  # Wait for the robot to stop
                        break

            # Additional processing...
        except Exception as e:
            print(f"An error occurred: {e}")
            continue
except KeyboardInterrupt:
    print("Data collection stopped by user.")
finally:
    con.send_pause()
    con.disconnect()

# Calculate the median loop frequency
def calculate_median(values):
    sorted_values = sorted(values)
    n = len(sorted_values)
    if n % 2 == 1:
        # If odd, return the middle element
        return sorted_values[n // 2]
    else:
        # If even, return the average of the two middle elements
        return (sorted_values[n // 2 - 1] + sorted_values[n // 2]) / 2.0

median_loop_frequency = calculate_median(loop_frequencies)
print(f"Median loop frequency: {median_loop_frequency:.2f} Hz")
#Median loop frequency: 791.08 Hz
t1 = time.time()
# Stop data acquisition
print("Stopping data acquisition...")
amti_dll.fmBroadcastStop()
print("Data acquisition stopped.")


# Create a figure and a set of subplots
fig, axs = plt.subplots(2, 1, figsize=(12, 12))

# Plot for Force Components
axs[0].plot(Fx_values, label='Fx', marker='o', markersize=1.5, linewidth=1)
axs[0].plot(Fy_values, label='Fy', marker='x', markersize=1.5, linewidth=1)
axs[0].plot(Fz_values, label='Fz', marker='^', markersize=1.5, linewidth=1)
axs[0].set_xlabel('Sample Number')
axs[0].set_ylabel(f'Force Value {force_units}')
axs[0].set_title('Force Components Fx, Fy, Fz')
axs[0].legend()
axs[0].grid(True)

# Plot for Moment Components
axs[1].plot(Mx_values, label='Mx', marker='o', markersize=1.5, linewidth=1)
axs[1].plot(My_values, label='My', marker='x', markersize=1.5, linewidth=1)
axs[1].plot(Mz_values, label='Mz', marker='^', markersize=1.5, linewidth=1)
axs[1].set_xlabel('Sample Number')
axs[1].set_ylabel(f'Moment Value {moment_units}')
axs[1].set_title('Moment Components Mx, My, Mz')
axs[1].legend()
axs[1].grid(True)

# Save the figure containing both plots
plt.tight_layout()  # Adjust the layout to make room for all subplots
output_FT_path = os.path.join(output_path, "force_moment_graphe.png")
plt.savefig(output_FT_path)
plt.show()




# Write the positions to a text file
with open(output_position_path, 'w') as p:
    for pos in tcp_pose_list:
        A = convert_coordinates(pos)        # Convert the coordinates to point A
        p.write(f"{A[0]:<10.5f}\t{A[1]:<10.5f}\t{A[2]:<10.5f}\t{A[3]:<10.5f}\t{A[4]:<10.5f}\t{A[5]:<10.5f}\n")

                

# Create a 3D scatter plot for positions
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Extract positions and orientations
positions = [tcp[:3] for tcp in tcp_pose_list]
orientations = [tcp[3:] for tcp in tcp_pose_list]
# Unpack positions into x, y, z
x, y, z = zip(*positions)
ax.scatter(x, y, z, c='b', marker='o')

# # Optionally, add quivers to represent orientations
# for pos, ori in zip(positions, orientations):
#     ax.quiver(pos[0], pos[1], pos[2], ori[0], ori[1], ori[2], length=0.1, normalize=True)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()

