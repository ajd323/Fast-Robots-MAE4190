+++
title = "Lab 1: Artemis and BLE"
date = 2026-02-03
description = ""
taxonomies = { tags = ["Artemis", "BLE", "Jupyter Notebook", "ArduinoIDE"] }
+++

## Summary (1A and 1B)

Lab #1 is a basic introduction of the Artemis Nano board and interacting with the Arduino IDE, through both direct connection and BLE. In Lab #1A, the Artemis board is configured for a MacBook (M2 Pro) and basic scripts are executed including Blink, Serial, AnalogRead, and Microphone Output. In Lab #1B, bluetooth communication is established through Jupyter Notebook to wirelessly communicate with the Artemis board, and basic verification testing scripts are conducted to confirm a reliable connection.

## Lab 1A

### Configurations
 Prior to beginning, some steps were required to configure my computer for communicating with the Artemis board. This included installing the Artemis library for the preinstalled ArduinoIDE, downloading the CH340 Driver, and configuring particular settings for the bootloader. After completing this, the ArduinoIDE could recognize and burn scripts.

### Outcomes
Multiple tests were conducted to verify the Artemis Nano board is operating as expected, as shown in the videos below:

**Info:** Test Function #1

```cpp
case SEND_THREE_FLOATS:
    float three_float_array[3];

    for (int i = 0; i < 3; i++) {
        success = robot_cmd.get_next_value(three_float_array[i]);
        if (!success) {
            return;
        }
    }

    Serial.print("Three Float Test: ");
    Serial.print(three_float_array[0]);
    Serial.print(", ");
    Serial.print(three_float_array[1]);
    Serial.print(", ");
    Serial.println(three_float_array[2]);
```

## Lab 1B

### Configurations
Prior to beginning the next lab, additional steps were completed to create the virtual environments required for running bluetooth for the Artemis Nano board. This included multiple steps, first including downloading python, creating the FastRobots_ble virtual environment with Python, and uploading the “ble_robot_1.4” codebase into the project directory. After this, the “ble_arduino.ino” script is loaded onto the board to extract the Artemis MAC address, generate a new UUID in Jupyter Notebook, and update the “connection.yaml” file with the new MAC address in the project directory. There are smaller steps beyond this to produce the first demo for bluetooth capabilities, but for brevity these are omitted.

Now that the BLE environment has been initialized, the codebase creates a two-way wireless Bluetooth Low Energy (BLE) communication channel designed for low power devices. BLE operates with a central-peripheral system, where the peripheral (i.e. the Artemis Nano board) advertises itself to the central (i.e. Main Computer) and provides structured information as services and characteristics. Each piece of information has a unique UUID which enables the computer to read and write characteristics, in addition to subscribing for updates. With this, a publish-subscribe system is accessible for wireless controlling the Artemis Nano and receiving sensor data through Bluetooth.

### Outcomes

## Discussion

Throughout this lab, I became familiar with the basic infrastructure for sending and receiving information through direct and wireless connections. This ranges from configuring the Artemis board, burning normal scripts, communicating with the Serial Monitor, creating the infrastructure for a virtual environment through Jupyter Notebook, and sending data through BLE. Major challenges included software blocks associated with using an Apple computer during the initial configuration, but beyond this, all future steps faced little issues. Overall, this lab was a strong start towards building the digital infrastructure for assembling the robots in the upcoming labs.