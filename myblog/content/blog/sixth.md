+++
title = "Lab 6: Orientation PID"
date = 2026-03-17
description = ""
taxonomies = { tags = ["Open-Loop Design", "Closed-Loop Design", "PID", "Wireless BLE"] }
+++

## Sumamry Lab #6

Lab #6 is an expansion of the linear Proportional-Integral-Deriative (PID) controller developed in the previous lab (Lab #5), integrating the closed-loop feedback system based on relative orientation components. By adapting the PID-based controller for wall collision control based on the stunt car’s yaw (measured with the IMU), the closed-loop system assists in maintaining consistent yaw through disturbances and offsets.

## Configuration

The stunt car configuration is still the same from all previous labs, including wired components, ToF locations, and overall assembly (with the 3D printed plates). Additionally, motor drivers were replaced between Lab #5 and Lab #6 due to accidental incorrect wiring for the battery. The following is an image of the stunt car and sensors for the lab:

*PHOTO*

**Sending and Receiving Bluetooth for PID**

A similar control format is integrated for the orientation-based PID system as the linear-based PID system. This function is natively set onto the RedBoard Artemis Nano and operates based on an on-off logic through a BLE command on the Jupyter Notebook. After this lab, PID control is split into two distinct types of control methods: linear and orientation. The previous code for wall-collision is renamed/ reworked to be labeled as the “Linear PID” and all set-up/ helper functions for this labelled “Orientation PID.” With this in mind, the following components are near-identical to the previous lab. The functions “void PID_step_ori” is initialized on the RedBoard Artemis Nano for controlling PID gain and motion with a looped step function:

*Arduino Code (C++)*
```cpp
   while (central.connected()) {
       write_data();
       read_data();
       if(lin_PID_control){
         unsigned long now = millis();
         if(now - lin_last_PID_time >= lin_PID_interval){
           lin_last_PID_time = now;
           PID_step_lin();
         }
       }
       if(ori_PID_control){
         unsigned long now = millis();
         if(now - ori_PID_last_time >= ori_PID_interval){
           ori_PID_last_time = now;
           PID_step_ori();
         }
       }
     }
void PID_step_ori() {
   // PID Control Code to Be Added
   if (output > 0) {
       // turn right
       analogWrite(Pin9, output * CF_Left);
       analogWrite(Pin11, 0);
       analogWrite(Pin12, 0);
       analogWrite(Pin13, output * CF_Right);
   } else {
       float rev = -output;
       // turn left
       analogWrite(Pin9, 0);
       analogWrite(Pin11, rev * CF_Right);
       analogWrite(Pin12, rev * CF_Left);
       analogWrite(Pin13, 0);
   }
}
```

Additionally, these functions are nested into a while True loop based on the boolean value “ori_PID_control,” which altered externally from BLE, python commands “case BEGIN LIN PID,” “case END LIN PID,” and “case ORI PID GAINS:” 

*Arduino Code (C++)*
```cpp
case BEGIN_ORI_PID: {
     bool Received_input = robot_cmd.get_next_value(ori_PID_target);
     // No signal loops the previous call request
     if(!Received_input) return;
     ori_integral_error = 0.0;
     ori_PID_prev_error = 0.0;
     ori_PID_last_time = millis();
     ori_PID_counter = 0;
     ori_PID_control = true;
     // Send Data to Python Terminal
     tx_estring_value.clear();
     tx_estring_value.append("Orientation PID Started, Target Set: ");
     tx_estring_value.append(ori_PID_target);
     tx_estring_value.append(" Degrees");
     tx_characteristic_string.writeValue(tx_estring_value.c_str());
     break;
   }
   case END_ORI_PID: {
     ori_PID_control = false;
     analogWrite(Pin12, 0);
     analogWrite(Pin13, 0);
     analogWrite(Pin9, 0);
     analogWrite(Pin11, 0);
     // Send Data to Python Terminal
     tx_estring_value.clear();
     tx_estring_value.append("Orientation PID Stopped");
     tx_characteristic_string.writeValue(tx_estring_value.c_str());
     break;
   }
   case PID_ORI_GAINS: {
     bool Received_input = robot_cmd.get_next_value(Kop)
                       && robot_cmd.get_next_value(Koi)
                       && robot_cmd.get_next_value(Kod);
     if(!Received_input) return;
     // Send Data to Python Terminal
     tx_estring_value.clear();
     tx_estring_value.append("Orientation PID Gains are Set, Ori. Kp = ");
     tx_estring_value.append(Kop);
     tx_estring_value.append(", Ori. Ki = ");
     tx_estring_value.append(Koi);
     tx_estring_value.append(", Ori. Kd = ");
     tx_estring_value.append(Kod);
     tx_characteristic_string.writeValue(tx_estring_value.c_str());
     break;
   }
```

*Jupyter Notebook (Python)*
```cpp
ble.send_command(CMD.PID_ORI_GAINS, "1.2|0.0|0.05")
ble.send_command(CMD.BEGIN_ORI_PID, "304")
ble.send_command(CMD.END_ORI_PID, "")
```

These two functions are intended for toggling the PID function on the board, and a third function “PID_ORI_TRANSMISSION” wirelessly streams time, position, and PID values for a given package size. This streaming occurs after each of the following arrays are the size of a predetermined variable saved as “time_package_size:”

*Arduino Code (C++)*
```cpp
case PID_ORI_TRANSMISSION: {
     for (int i = 0; i < ori_PID_counter; i++) {
       tx_estring_value.clear();
       tx_estring_value.append("Time (ms): ");
       tx_estring_value.append(ori_timestamp_value[i]);
       tx_estring_value.append(",");
       tx_estring_value.append("Motor PWM: ");
       tx_estring_value.append(ori_motorPWM_value[i]);
       tx_estring_value.append(",");
       tx_estring_value.append("Distance: ");
       tx_estring_value.append(ori_distance_value[i]);
       tx_estring_value.append(",");
       tx_estring_value.append("Proportional Error: ");
       tx_estring_value.append(ori_proportional_value[i]);
       tx_estring_value.append(",");
       tx_estring_value.append("Integral Error: ");
       tx_estring_value.append(ori_integral_value[i]);
       tx_estring_value.append(",");
       tx_estring_value.append("Derivative Error: ");
       tx_estring_value.append(ori_derivative_value[i]);
       tx_characteristic_string.writeValue(tx_estring_value.c_str());
     }
     break;
   }
   case ORI_SAMPLE_RATE:{
     float new_rate_ms;
     bool Received_input = robot_cmd.get_next_value(new_rate_ms);
     if (!Received_input) return;
     ori_PID_interval = (unsigned long)new_rate_ms;
     tx_estring_value.clear();
     tx_estring_value.append("Sample rate updated to ");
     tx_estring_value.append((int)ori_PID_interval);
     tx_estring_value.append(" ms");
     tx_characteristic_string.writeValue(tx_estring_value.c_str());
     break;
   }
```

*Jupyter Notebook (Python)*
```cpp
time_package_size = 5000 # Consistent with Arduino
timestamp_list = []
measurement_list = []
proportional_list = []
integral_list = []
derivative_list = []

def notif_callback_PID(UUID, Notif_Array):
    input_data = ble.bytearray_to_string(Notif_Array)
    processed_data = input_data.split(",")
    print(processed_data)
    timestamp_list.append(float(processed_data[0].split(": ")[1]))
    measurement_list.append(float(processed_data[1].split(": ")[1]))
    proportional_list.append(float(processed_data[2].split(": ")[1]))
    integral_list.append(float(processed_data[3].split(": ")[1]))
    derivative_list.append(float(processed_data[4].split(": ")[1]))
    if len(timestamp_list) >= time_package_size:
        print("Data collection complete!")

ble.start_notify(ble.uuid['RX_STRING'], notif_callback_PID)
ble.send_command(CMD.PID_ORI_TRANSMISSION, "")
```

**Digital Motion Processing (DMP) Enabling**

In addition to the previous set-up, the Digital Motion Processing (DMP) is established for the ICM-20948 9DOF breakout board to reduce drift experienced in previous labs. This is accomplished by comparing the data between the gyroscope, the accelerometer, and the magnetometer to correct error and improve the orientation measurement.

To enable the DMP, the “ICM_20948_C.h” file in the SparkFun ICM-20948 Arduino Library is edited to provide access to the DMP feature. By commenting out the “#define ICM_20948_UES_DMP” definition, the DMP feature is accessible for all future usages. “Example7_DMP_Quat6_EulerAngles” is uploaded to the RedBoard Artemis Nano to observe the improved IMU orientation measurements (pitch, roll, yaw).

*VIDEO*

## Lab #6 Outcomes

**P/I/D Discussion**

With the DMP initiated, the final closed-loop feedback controller developed for the orientation-based is a variation of the previous linear PID controller. The script receives data regarding the ideal orientation from the “PID_ORI_GAIN” function, and the main orientation controller utilizes data from the IMU to adjust the relative yaw. The PID values and response is stored locally on the Artemis Nano board, and the “PID_ORI_TRANSMISSION” function sends the data to the Jupyter Notebook script for graphing. The following are the major attributes of the orientation-based PID controller on the Artemis Nano board: 

*Arduino Code (C++)*



## Lab #6 Outcomes

This lab was a great chance to further improve the PID-controller from Lab #5 for orientation-based control. The tuning for the individual gain controllers for proportional, integral, and derivative constants provided a strong in-depth analysis for their individual contributions to motor dynamics. This lab was completed with Jamison Taylor, assisted from AI tools for minor debugging, and referenced against Tyler Wisnieski’s GitHub page for comparing effective code structures.