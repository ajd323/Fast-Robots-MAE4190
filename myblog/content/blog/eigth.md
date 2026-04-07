+++
title = "Lab 8: Stunts!"
date = 2026-04-07
description = ""
taxonomies = { tags = ["Closed PID", "Programmed Motion"] }
+++

## Summary Lab #8

Lab #8 is an opportunity to test the functionality of the linear/orientation PID controllers, the Kalman filters, and sensor sampling in a real-world example. With the architecture created from Lab #5 to Lab #7, a reliable flip and drift function is constructed. From this

## Lab #8 Outcomes

### Task A: Stunt Car Flip

The flip functions has the following requirements to be considered successful from Lab #8:
- Start at a designated line <4 m away from the wall
- Perform a flip ~1 ft from the wall
- Drive back in the original direction

With this in mind, the following BLE case, "Flip," is designed to handle these requirements. Here is the major attribtues of the code packages in the Artemis Nano board.

First, additional BLE cases are added to the Artemis Board for conducting and transmitting data for the flip jump.
```cpp
enum CommandTypes{
    ...
    FLIP_STUNT,
    FLIP_TRANSMISSION,
}
```

From this, the BLE case for flip is created that uses a step structure similar to previous PID linear and orientation functions:
```cpp
case FLIP_STUNT:{
    flip_active = true;
    flip_state = 0;
    flip_counter = 0;
    flip_state_start = millis();
    flip_last_log = flip_state_start;
    tx_estring_value.clear();
    tx_estring_value.append("Flip stunt started");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    break;
}
case FLIP_TRANSMISSION:{
    for (int i = 0; i < flip_counter; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("T (ms): ");
        tx_estring_value.append(lin_timestamp_value[i]);
        tx_estring_value.append(",");
        tx_estring_value.append("PWM: ");
        tx_estring_value.append(lin_motorPWM_value[i]);
        tx_estring_value.append(",");
        tx_estring_value.append("D: ");
        tx_estring_value.append(lin_distance_value[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
}
```

Finally, a "flip_step" function is implemented that controls the forward motion and toggling the orientation PID:

```cpp
void flip_step() {
    if (!flip_active) return;
    unsigned long now = millis();
    float dist = (float)ToF_sensor_2.read();
    if (now - flip_last_log >= 20) {
        float pwm = 0.0;
        if (flip_counter > 0) {
            pwm = lin_motorPWM_value[flip_counter - 1];  // last PWM value
        }
    if (flip_counter < time_package_size) {
        lin_timestamp_value[flip_counter] = now - flip_state_start;
        lin_motorPWM_value[flip_counter] = pwm;
        lin_distance_value[flip_counter] = dist;
        lin_kfdist_value[flip_counter] = 0;
        lin_kfVel_value[flip_counter] = 0;
        flip_counter++;
    }
    flip_last_log = now;
    }
    if (flip_state == 0) {
        Serial.println("0");
        yaw_start = gyro_yaw;  // or DMP yaw if using DMP
        ori_PID_target = yaw_start;
        ori_PID_control = false;
        flip_state = 1;
        flip_state_start = now;
    }
    else if (flip_state == 1) {
        Serial.println("1");
        analogWrite(Pin12, 255 * CF_Left);
        analogWrite(Pin13, 255 * CF_Right);
        analogWrite(Pin9, 0);
        analogWrite(Pin11, 0);
        dist = (float)ToF_sensor_2.read();
        if (dist <= flip_trigger_dist_mm) {
            analogWrite(Pin9, 255 * CF_Left);
            analogWrite(Pin11, 255 * CF_Left);
            analogWrite(Pin12, 0);
            analogWrite(Pin13, 0);
            delay(100);
            ori_PID_control = true;
            flip_state = 2;
            flip_state_start = now;
        }
    }
    else if (flip_state == 2) {
        Serial.println("2");
        float yaw_err = fabs(ori_PID_prev_error);
        if (yaw_err < 5.0) {  // within tolerance
            ori_PID_control = false;
            flip_state = 3;
            flip_state_start = now;
        }
    }
    else if (flip_state == 3) {
        Serial.println("3");
        analogWrite(Pin12, 255 * CF_Left);
        analogWrite(Pin13, 255 * CF_Right);
        analogWrite(Pin9, 0);
        analogWrite(Pin11, 0);
        if (dist >= flip_return_dist_mm) {
            analogWrite(Pin9, 0);
            analogWrite(Pin11, 0);
            analogWrite(Pin12, 0);
            analogWrite(Pin13, 0);
            flip_state = 4;
            flip_active = false;
        }
    }
}
```

*Flip Test Success Trial #1*

*VIDEO 1*

*GRAPH 1*

### Task B: Stunt Car Drift

The drift functions has the following requirements to be considered successful from Lab #8:
- Start at a designated line <4 m away from the wall
- Initiate a 180 degree turn when within 3ft of the wall

With this in mind, the following BLE case, "Drift," is designed to handle these requirements. Here is the major attribtues of the code packages in the Artemis Nano board.



*Drift Test Success Trial #1*

*VIDEO 1*

*GRAPH 1*

## Lab #8 Discussion

This lab is a great opportunity to have fun with the mechanics of the car in a more objective-based project. By focusing on the flip and drift outcomes, this provides more freedom with designing the 