# Ubuntu 20.04 (OS) Installation
1. Download [Ubuntu 20.04 Focal](https://releases.ubuntu.com/focal/) Image (64bit PC AMD64)
2. Create a bootable image of the downloaded file using [BalenaEtcher](https://etcher.balena.io/#download-etcher)
3. Reboot your *Desktop/Laptop* and press *F12/F2* depending to load the **Boot Menu**
4. Follow the tutorials to install Ubuntu
5. You should be able to login to the installed Ubuntu by the next reboot

# CUDA (GPU Compatibility) Installation

- Prerequisites:
    - Having a CUDA Capable **Nvidia Graphical Processor Unit (GPU)**
    - The **GPU** should be compatible with:
        - *CUDA>=11.0*
        - *CUDA ToolKit = 535.183.01*
            - Tested GPUs: *RTX 2070S, 4070TI, 4090*
    - Having a Beginner to Intermediate knowledge of working with **Ubuntu Command Terminal**

- Installing CUDA:
    - [CUDA 11.8](https://developer.nvidia.com/cuda-11-8-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_local)
    - Follow the provided terminal commads provided in the link
        ```shell
        wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
        sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
        wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb
        sudo dpkg -i cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb
        sudo cp /var/cuda-repo-ubuntu2004-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/
        sudo apt-get update
        sudo apt-get -y install cuda
        ```
    - Open ~/.bashrc
        ```shell
        sudo nano ~/.bashrc
        ```
    - Add these lines to the end of the file and press **Ctrl+X** to save
        ```shell
        export PATH=/usr/local/cuda-11.8/bin${PATH:+:${PATH}}
        export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
        ```

Following the provided terminal commands **(Might be Outdated, so its better to use NVIDIA website's link than the provided command lines)** you should be able to install CUDA, but some of the packages might show installation error **(Which is because of the outdated Cuda Toolkit installation alongside with CUDA 11.8)**
