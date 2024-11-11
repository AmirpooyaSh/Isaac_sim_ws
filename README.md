# University of Florida - Smart IDC Lab Repository
This repository belongs to the Smart IDC Lab (supervised by Dr. Aladdin Alwisy - M.E. Rinker School of Construction)

# Requirements
- Ubuntu 22.04.5 LTS
- CUDA = 11.8
- NVIDIA GPU (Your device's GPU should be able to run on NVIDIA Driver >= 450.80.02)

    <div style="text-align: center;">
        <img src="https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/imgs/gpu_capability.png" alt="GPU Capability" />
    </div>
<br>
    To make sure, you can run the command below and check for the availability:

        ```shell
        sudo add-apt-repository ppa:graphics-drivers/ppa
        sudo apt update
        ubuntu-drivers devices
        ```
<br>
    Any NVIDIA driver >= 450-80-02 means that CUDA 11.8 can work with your GPU's NVIDIA driver

- NVIDIA Driver >= 535.183.01 **(Tested with 535.183.01 (RTX 2070S), 550.127.05 (RTX 4070TI), 565.57.01 (RTX 4090))**
- Isaac Sim = 4.2.0
- ROS2 / ROS Noetic (Docker Build)
- CuRobo 0.7.4

# Installation

Following the given tutorials in the `/doc` directory, you should be able to work with the package:

1. [**Hardware Configuration**](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/Hardware_Config/Ubuntu_Cuda_VScode.md) (Ubuntu + CUDA + VSCode)
2. [**Isaac Sim Installation**](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/Hardware_Config/Isaac_Sim.md)
3. [**CuRobo Installation**](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/Hardware_Config/CuRobo_Install.md)
4. [**Running IDC Lab's Provided Example**](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/Setup/IDC_Lab_Example.md)


# Developemental Tutorials

After testing the provided example, the following tutorials will let you understand the logic behind the provided developement (you should be able to create your own world following the provided sections)

1. [**Robot Setup** (CuRobo)](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/Setup/Robot_Setup.md)
2. [**Robot Object Explanation** (Can be Expanded to any Robotic Arm) - *Dual IRB6620 (11/08/2024)*](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/Setup/Robot_Object.md)


# Author
Amirpooya Shirazi
