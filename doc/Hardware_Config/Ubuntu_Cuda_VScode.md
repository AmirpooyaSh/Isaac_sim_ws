# Ubuntu 20.04 (OS) Installation
1. Download [Ubuntu 20.04 Focal](https://releases.ubuntu.com/focal/) Image (64bit PC AMD64)
2. Create a bootable image of the downloaded file using [BalenaEtcher](https://etcher.balena.io/#download-etcher)
    - Make sure to turn off **Secure Boot** on your device using **BIOS Setting** before proceeding to the installation
    - Turn off **RTS/RAID on** on your device using **BIOS Setting**
    - If you're doing a dual boot **Windows/Ubuntu**, make sure to follow [this tutorial](https://www.youtube.com/watch?v=YRu__8ggMSY) before proceeding to the installation
3. Reboot your **Desktop/Laptop** and press **F12/F2** depending to load the **Boot Menu**
4. Follow the tutorials to install Ubuntu
5. You should be able to login to the installed Ubuntu by the next reboot

<br>

# CUDA (GPU Computation Library) Installation

- Prerequisites:
    - Having a CUDA Capable **Nvidia Graphical Processor Unit (GPU)**
    - The **GPU** should be compatible with:
        - *CUDA>=11.0*
        - *CUDA ToolKit = 535.183.01*
            - Tested GPUs: *RTX 2070S, 4070TI, 4090*
    - Having a Beginner to Intermediate knowledge of working with **Ubuntu Command Terminal**

<br><br>

- Installing CUDA:

    - [CUDA 11.8](https://developer.nvidia.com/cuda-11-8-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_local)
    - Follow the provided commads **(Same as the ones provided link)**

        ```shell
        wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
        sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
        wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb
        sudo dpkg -i cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb
        sudo cp /var/cuda-repo-ubuntu2004-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/
        sudo apt-get update
        sudo apt-get -y install cuda
        ```
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

<br><br>

Following the provided terminal commands **(might be outdated, so its better to use NVIDIA website's link than the provided command lines)** you should be able to install CUDA, but some of the packages might show installation error **(which is because of the outdated Cuda Toolkit installation alongside with CUDA 11.8)**.

- Updating the **CUDA Toolkit**:

    - Remove Outdated CUDA DKMS files:

        ```shell
        sudo rm -r /var/lib/dkms/nvidia
        ```
    - Remove Outdated ToolKit:

        ```shell
        sudo apt-get purge 'nvidia-*'
        ```
    - Reinstall basic Ubuntu DKMS files:

        ```shell
        sudo apt-get install build-essential dkms
        sudo apt-get install linux-headers-$(uname -r)
        ```
    - Reboot the device **(Read the following section before Rebooting)**

<br><br>

Now that you uninstalled the outdated **CUDA Toolkit** and its **Booting Kernels** ubuntu won't come up by itself and you will see a black screen, because the OS tries to boot using GPU interface, but there is no GPU software to support that. At this point you should boot your OS with its default kernel **nomodeset**.

- Booting Ubuntu on default Kernel:

    - On boot menu **(GRUB Menu)** press **"e"**
    - Add the word **nomodeset** to the line before the ending line which starts with:

        ```shell
        linux        /boot/...........                      nomodeset
        ```
    
    - Press **Ctrl+X** to boot into Ubuntu

<br><br>

Now you have to install the newer version of **CUDA Toolkit** to make things work again.

- Installing **CUDA Toolkit 535.183.01**

    - Open a terminal and write:
        
        ```shell
        sudo apt-get install nvidia-driver-535
        sudo update-initramfs -u
        ```
    
    - Reboot the device

<br><br>

If you've done everything as mentioned, Ubuntu should come up without any preprocesses. Once it came up, check the **CUDA Toolkit** version by:

    nvidia-smi

<br><br>

**If you get something like below, you're all fine !!!!!**

![CUDA Toolkit check](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/imgs/cuda_toolkit.png)

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

