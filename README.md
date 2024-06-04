# F1TENTH_PROJECT - Firmware

### 2.1. Install the Editor
- Download and install STM32CubeIDE from [this link](https://www.st.com/en/development-tools/stm32cubeide.html).

### 2.2. Setup the Environment
To set up the environment, follow these steps:
1. Clone the repository:
    ```sh
    git clone https://github.com/tanakon-apit/F1TENTH_PROJECT.git
    ```
2. Navigate to the project directory:
    ```sh
    cd F1TENTH_PROJECT/STM32L432KC_F1TENTH_FIRMWARE/
    ```
   The directory `F1TENTH_PROJECT/STM32L432KC_F1TENTH_FIRMWARE/` contains the project files.

3. Set up micro-ROS for the STM32 project by following the instructions [here](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils).

### 2.3. Build the Firmware
Before building the firmware, ensure you have the necessary permissions to access Docker by running:
```sh
sudo chmod 666 /var/run/docker.sock
```
Once this command is executed, you can proceed to build the firmware by clicking the build button in STM32CubeIDE.

### 2.4. Important Notice
If the micro-ROS agent fails to start, verify the connection of the sensor. The agent will not initialize if the sensor is not connected properly.
