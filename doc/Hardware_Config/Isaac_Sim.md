# Isaac Sim Installation (4.2.0)

For the purpose of this research project, Isaac Sim 4.2.0 has been selected. Therefore, the provided instructions might be outdated and require you to dig further into this section (as of 11/08/2024).

- Sign up to [Nvidia Platform](https://static-login.nvidia.com/service/default/index.html?redirect_uri=https%3A%2F%2Flogin.nvidia.com%2Fcallback%2Femail&state=4eNU0nw8KemgxniKIIIEFWCFuSUI1On9P39YiUeWej2cLy8moNq8bFDs3Zn3w0MGH8L6fUdbH1WJFdnsIRFoSw&ui_locales=en)

- Download [Omniverse Linux Image](https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage)

- Move the downloaded file to somewhere more handy **I usually copy it to Desktop**

- Give it **Sudo Read/Write Access**

    ```shell
    sudo chmod 777 omniverse-launcher-linux.AppImage
    ```

- Double Click on it to launch the OmniverLauncher

- Login to your created Nvidia account

- Install the following applications by heading to **"Exchange->Search"**

    ![Isaac Sim Exchange](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/imgs/isaac_download.png)

    - Omniverse Cache

    - Omniverse Nucleus **(It's mostlikely installed, but you have to set it up)**
        
        - Use the [provided link](https://docs.omniverse.nvidia.com/nucleus/latest/workstation/installation.html) to set the Nucleus application
    
    - Omniverse Isaac Sim Compatibility Checker **(Running it will check the CUDA+Toolkit requirements)**

        - If everything gone **Green** then you're good to install Isaac Sim

            ![Isaac Sim Compat](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/imgs/isaac_compat.png)

    - Omniverse Isaac Sim
