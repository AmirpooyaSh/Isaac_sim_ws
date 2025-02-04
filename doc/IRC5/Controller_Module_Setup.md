# IRC5 Module Setup

Navigate to the **/Isaac_sim_ws/IRC5_Setup**. Within the folder, you'll find the required IRC5 controller Backup. By restoring this backup into the controller, you'll
be able to send motion/io commands to it using python.

# Detailed Instructions

A. Get trusted and connect to the local network the IRC5 controller is connected to.

- To be able to get trusted on the network **(SIDCf)**, you should contact **Dr. Aladdin Alwisy" to give access to your device.
- Once your device got trusted in the network, check if you can see the IRC5 controller:

    ```shell
    ping 192.168.1.4
    ```
- If there were no timeouts, it means that your device can communicate to the IRC5.

B. 