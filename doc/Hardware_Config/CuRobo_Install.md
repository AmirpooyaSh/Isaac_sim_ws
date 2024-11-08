# CuRobo Installation (In Isaac Sim Environment)

It's worth mentioning that even though Isaac Sim is installed on your Ubuntu 20.04 OS, it's working environment is different. Isaac Sim is running the developed materials using **Python Virtual Enviroment** which is located at:

```shell
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh
```

Therefore, to use any additional library in Isaac Sim, it's essential to install it in **Isaac Sim's Virtual Environment** rather than your device. As a practical example, this section demonstrates the installation of **CuRobo** library in Isaac Sim's virtual enviroment which can be applied to any other library if needed.

As a rule of thump, Isaac Sim's virtual environment is going to be set as **"omni_python"** from now on, and it should be set in the **~/.bashrc** file by executing the below command:

echo "alias omni_python='~/.local/share/ov/pkg/isaac_sim-4.0.0/python.sh'" >> ~/.bashrc
