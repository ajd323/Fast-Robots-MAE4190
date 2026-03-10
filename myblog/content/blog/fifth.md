+++
title = "Lab 5: Linear PID and Linear Interpolation"
date = 2026-03-10
description = ""
taxonomies = { tags = ["Open-Loop Design", "Closed-Loop Design", "PID", "Wireless BLE"] }
+++

*Please Note: For transparency, this lab was submitted one day late due to issues optimizing the deadband filter in addition to selecting the proper Kp, Ki, and Kd values*

## Sumamry Lab #5

Lab #5 integrates a closed-loop feedback system for controlling the stunt car linear motion and collision detection. Where Lab #4 ended by introducing the open-loop control system for motion, Lab #5 expands by utilizing data from the Time of Flight (ToF) sensors to construct a more complex closed-loop system known as a Proportional-Integral-Derivative (PID) controller. By tuning the various gain constants for the typical PID controller framework, the stunt car is enabled with improved handling while stopping for obstacles.

## Configuration

The stunt car follows the same configurations as the previous lab, including wired components, ToF locations, and overall assembly. Additionally, a custom 3D-printed plate was installed for improved mounting of the Artemis Nano, IMU sensor, and the 4-port connector. The following are images of the stunt car and sensors for the following lab:

**Sending and Receiving Bluetooth for PID**

The PID controller is structured by formatting the control function natively onto the RedBoard Artemis Nano and operating on-off logic with a BLE command through Python. On the Artemis Nano, the “PID_step” function is initialized in C++ for computing the instantaneous PID gains and motion:

*Arduino Code (C++)*
```cpp
bool PID_control = false;
…
while (central.connected()) {
  write_data();
  read_data();
  if(PID_control){
      PID_step();
  }
…
void PID_step(){
// Code added later for specific PID motion
…
 if (motor_output > 0) {
   analogWrite(Pin9, motor_output * CF_Left);
   analogWrite(Pin11, motor_output * CF_Right);
   analogWrite(Pin12, 0);
   analogWrite(Pin13, 0);
 } else {
   motor_output = -motor_output;
   analogWrite(Pin12, motor_output * CF_Left);
   analogWrite(Pin13, motor_output * CF_Right);
   analogWrite(Pin9, 0);
   analogWrite(Pin11, 0);
 }
}
```

This function is nested into a while True loop based on the boolean value “PID_control,” which is altered externally from BLE, python commands “BEGIN_PID” and “END_PID.” In addition to these functions, “PID_GAINS” is provided to control the respective gain values, K, for the proportional, integral, and derivative components outside of beginning and ending the device:

*Arduino Code (C++)*
```cpp
case BEGIN_PID: {
     bool Received_input = robot_cmd.get_next_value(PID_target);
     // No signal loops the previous call request
     if(!Received_input) return;
     integral_error = 0.0;
     PID_prev_error = 0.0;
     PID_last_time = millis();
     PID_counter = 0;
     PID_control = true;
     // Send Data to Python Terminal
     tx_estring_value.clear();
     tx_estring_value.append("PID Started, Target Set: ");
     tx_estring_value.append(PID_target);
     tx_estring_value.append(" mm");
     tx_characteristic_string.writeValue(tx_estring_value.c_str());
     break;
   }
  case END_PID: {
     PID_control = false;
     analogWrite(Pin12, 0);
     analogWrite(Pin13, 0);
     analogWrite(Pin9, 0);
     analogWrite(Pin11, 0);
     // Send Data to Python Terminal
     tx_estring_value.clear();
     tx_estring_value.append("PID Stopped");
     tx_characteristic_string.writeValue(tx_estring_value.c_str());
     break;
   }
case PID_GAINS: {
     bool Received_input =  robot_cmd.get_next_value(Kp)
                       && robot_cmd.get_next_value(Ki)
                       && robot_cmd.get_next_value(Kd);
     if(!Received_input) return;
     // Send Data to Python Terminal
     tx_estring_value.clear();
     tx_estring_value.append("PID Gains are Set, Kp = ");
     tx_estring_value.append(Kp);
     tx_estring_value.append(", Ki = ");
     tx_estring_value.append(Ki);
     tx_estring_value.append(", Kd = ");
     tx_estring_value.append(Kd);
     tx_characteristic_string.writeValue(tx_estring_value.c_str());
     break;
   }
```
*Jupyter Notebook (Python)*
```cpp
ble.send_command(CMD.PID_GAINS, "1.2|0.0|0.05")
ble.send_command(CMD.BEGIN_PID, "304")
ble.send_command(CMD.END_PID, "")
```

These two functions are intended for toggling the PID function on the board, and a third function “PID_TRANSMISSION” wirelessly streams time, position, and PID values for a given package size. This streaming occurs after each of the following arrays are the size of a predetermined variable saved as “time_package_size:”

*Arduino Code (C++)*
```cpp
case PID_TRANSMISSION: {
     for (int i = 0; i < PID_counter; i++) {
       tx_estring_value.clear();
       tx_estring_value.append("Time (ms): ");
       tx_estring_value.append(timestamp_value[i]);
       tx_estring_value.append(",");
       tx_estring_value.append("Distance: ");
       tx_estring_value.append(distance_value[i]);
       tx_estring_value.append(",");
       tx_estring_value.append("Proportional Error: ");
       tx_estring_value.append(proportional_value[i]);
       tx_estring_value.append(",");
       tx_estring_value.append("Integral Error: ");
       tx_estring_value.append(integral_value[i]);
       tx_estring_value.append(",");
       tx_estring_value.append("Derivative Error: ");
       tx_estring_value.append(derivative_value[i]);
       tx_characteristic_string.writeValue(tx_estring_value.c_str());
     }
     break;
```

*Jupyter Notebook (Python)*
```cpp
time_package_size = 5000 # Consistent with Arduino
timestamp_list = []
distance_list = []
proportional_list = []
integral_list = []
derivative_list = []

def notif_callback_PID(UUID, Notif_Array):
    input_data = ble.bytearray_to_string(Notif_Array)
    processed_data = input_data.split(",")
    print(processed_data)
    timestamp_list.append(float(processed_data[0].split(": ")[1]))
    distance_list.append(float(processed_data[1].split(": ")[1]))
    proportional_list.append(float(processed_data[2].split(": ")[1]))
    integral_list.append(float(processed_data[3].split(": ")[1]))
    derivative_list.append(float(processed_data[4].split(": ")[1]))
    if len(timestamp_list) >= time_package_size:
        print("Data collection complete!")

# Start notifications
ble.start_notify(ble.uuid['RX_STRING'], notif_callback_PID)
ble.send_command(CMD.PID_TRANSMISSION, "")
```

## Lab #4 Outcomes

**P/I/D Discussion**

The final closed-loop feedback controller function developed for usage during the lab is a complete PID controller controlled with the aforementioned BLE commands. The script receives data from the front ToF sensor in addition to the final distance from local objects, and calculates the respective instantaneous gain values for each of the controller components. The PID values are stored locally on the Artemis board for the “PID_TRANSMISSION” function. These were developed individually, however, are compiled and separated with comments for cohesion and clarity. The following are the major attributes of the coding structure on the Artemis Nano board:

*Arduino Code (C++)*
```cpp
void PID_step(){
 // Setting Time Steps and variables
 float ToF_Front = ToF_sensor_2.read();
 unsigned long now = millis();
 // Derivative Time Steps for Derivative Value 
 float dt = (now - PID_last_time) / 1000.0;
 if(dt <= 0) return;
 PID_last_time = now;
 // Calculate Proportional, Integral, and Derivative Error
 float proportional_error = PID_target - ToF_Front;
 integral_error += proportional_error * dt;
 float derivative_error = (proportional_error - PID_prev_error) / dt;
 PID_prev_error = proportional_error;
 // Output Calculation
 float motor_output =  Kp*proportional_error +
                       Ki*integral_error +
                       Kd*derivative_error;
 motor_output = constrain(motor_output, -255, 255);
 // Store Values
 if (PID_counter < time_package_size) {
   distance_value[PID_counter] = ToF_Front;
   timestamp_value[PID_counter] = now;
   proportional_value[PID_counter] = proportional_error;
   integral_value[PID_counter] = integral_error;
   derivative_value[PID_counter] = derivative_error;
   PID_counter++;
 }
…
}
```

For tuning the ideal constants for common usage with the stunt car, each component of the PID controller was tuned and progressively integrated starting from proportion gain (i.e. determine P, then PI, and finally PID). The ideal distance is set to “304 mm,” or about the distance of 1 tile (marked in the videos), and motor output is reduced in half to better tune the specific gain constants (i.e. “analogWrite(PinX, motor_output/2).
