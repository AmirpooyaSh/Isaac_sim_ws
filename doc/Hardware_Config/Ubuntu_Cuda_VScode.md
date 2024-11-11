# Ubuntu 20.04 (OS) Installation
1. Download [Ubuntu 22.04.5 LTS (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/) Image (64bit PC AMD64)
2. Create a bootable image of the downloaded file using [BalenaEtcher](https://etcher.balena.io/#download-etcher)
    - Make sure to turn off **Secure Boot** on your device using **BIOS Setting** before proceeding to the installation
    - Turn off **RTS/RAID on** on your device using **BIOS Setting**
    - If you're doing a dual boot **Windows/Ubuntu**, make sure to follow [this tutorial](https://www.youtube.com/watch?v=YRu__8ggMSY) before proceeding to the installation
3. Reboot your **Desktop/Laptop** and press **F12** depending to load the **Boot Menu**
4. Follow the tutorials to install Ubuntu
    - In the process of installing Ubuntu, connect your device to the internet
    - in the Installation section, make sure to check:
        - **Normal installation**
        - **Download updates while installting ubuntu**
        - **Install third-party software for ...**
        ![Ubuntu_Installation](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/imgs/ubuntu_install.png)
5. You should be able to login to the installed Ubuntu by the next reboot
6. You should be able to check the installed NVIDIA Driver by running the command below **(This is the driver Ubuntu installer decided to install which is the most recommended one for your GPU and should not be removed/changed manually)**:
    ```shell
    nvidia-smi
    ```
    ![CUDA Toolkit check](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/imgs/cuda_toolkit.png)


<br>

# CUDA (GPU Computation Library) Installation

- Prerequisites:
    - Having a CUDA Capable **Nvidia Graphical Processor Unit (GPU)**
    - The **GPU** should be compatible with:
        - *CUDA>=11.0*
    - Having a Beginner to Intermediate knowledge of working with **Ubuntu Command Terminal**

<br>

- Installing CUDA:

    - [CUDA 11.8](https://developer.nvidia.com/cuda-11-8-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=runfile_local)
    - Follow the provided commads **(Same as the ones provided link)**

        ```shell
        wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run
        sudo sh cuda_11.8.0_520.61.05_linux.run
        ```
        - Through the installation process, make sure to hit **Continue** instead of Abort and uncheck the installation of NVIDIA Driver **520.61.05**
        - The suggesting NVIDIA Driver **(520.61.05)** is providing an outdated installation which can lead into **Ubuntu Boot Problems** if you install it
        <br>
        ![CUDA_TK](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/imgs/cuda_tk_install.png)
    - Open ~/.bashrc by:

        ```shell
        sudo nano ~/.bashrc
        ```
    - Add these lines to the end of the file and press **Ctrl+X** to save:

        ```shell
        export PATH=/usr/local/cuda-11.8/bin${PATH:+:${PATH}}
        export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
        ```
    - Reboot your device and you should be able to get CUDA's version by typing:

        ```shell
        nvcc --version
        ```

        - Expected Result:

            ![CUDA Version Test](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/imgs/cuda.png)

If **nvidia-smi** and **nvcc --version** commands are working, then you're all set.

<br>

# VScode (Just For Developing Codes)

- Download the **Debian** version of VScode using this link: [VScode-Installer](https://code.visualstudio.com/download)

- Double click on the downloaded file and let it get installed

- You can check the installation by searching **"VSCode"** at this point

- Useful Extensions on VSCode:

    - Python
    - Pylance
    - C/C++
    - C/C++ Extension Pack
    - C/C++ Themes
    - CMake
    - CMake Tools
    - Docker **(Docker Developement Purposes)**
    - Dev Containers **(Docker Developement Purposes)**
    - Rainbow CSV **(Optional)**

<br>

# Enclosure

The provided installation guides are essential to learn/develope algorithms with **ROS + Isaac Sim**. Therefore, make sure to complete all the steps in this section before getting into the next section.

<br>

# What's Next?

[Isaac Sim Installation =>](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/Hardware_Config/Isaac_Sim.md) 

