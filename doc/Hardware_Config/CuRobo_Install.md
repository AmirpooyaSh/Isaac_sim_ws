# CuRobo Installation

It's worth mentioning that even though Isaac Sim is installed on your Ubuntu 20.04 OS, it's working environment is different. Isaac Sim is running the developed materials using **Python Virtual Enviroment** which is located at:

```shell
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh
```

Therefore, to use any additional library in Isaac Sim, it's essential to install it in **Isaac Sim's Virtual Environment** rather than your device. As a practical example, this section demonstrates the installation of **CuRobo** library in Isaac Sim's virtual enviroment which can be applied to any other library if needed.

As a rule of thump, Isaac Sim's virtual environment is going to be set as **"omni_python"** from now on, and it should be set in the **~/.bashrc** file by executing the below command:

```shell
echo "alias omni_python='~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh'" >> ~/.bashrc
```
<br>

# CuRobo Installation in Isaac Sim's Virtual Environment

- Execute the following commands **(Dependencies Installation)**:

    ```shell
    sudo apt install git-lfs
    source ~/.bashrc
    omni_python -m pip install tomli wheel ninja
    ```
- Clone and Install CuRobo:

    ```shell
    git clone https://github.com/NVlabs/curobo.git
    cd curobo
    omni_python -m pip install -e .[isaacsim] --no-build-isolation
    ```
- Check if CuRobo is installed by executing this command **(You Should Be In CuRobo Subdirectory)**:

    ```shell
    omni_python examples/isaac_sim/motion_gen_reacher.py --robot franka
    ```
    ![Franka Motion Generator](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/imgs/franka.png)

You can hit the play bottun and move the **Red Cube** to make the franka robot move.

# Enclosure

[Reference Link 1](https://curobo.org/get_started/1_install_instructions.html) 

<br>

[Reference Link 2](https://curobo.org/get_started/2b_isaacsim_examples.html)

<br>

# What's Next

[Running IDC Lab's Provided Example =>](https://github.com/AmirpooyaSh/Isaac_CuRobo/blob/main/doc/Setup/IDC_Lab_Example.md)