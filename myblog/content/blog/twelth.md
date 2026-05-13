+++
title = "Lab 12: Inverted Pendulum"
date = 2026-05-13
description = ""
taxonomies = { tags = ["PID Controller", "Stunt", "Final Lab"] }
+++

## Summary Lab #12

Lab #12 is a culmination of the previous labs for both electromechanical design and software development to execute a higher-level stunt. The inverted pendulum is a common PID-controller project, and integrated this system on the stunt car invovles leveraging previvous PID labs (5/6), Kalman Filtering labs (7), and stunts.The stunt car in this lab runs a basic PID-controlled inverted pendulum from rest at a vertical position, successfully re-orienting itself back to equilibrium. However, minor issues prevent the system from perfectly and indefinately maintaining equilobrium, likely a result of mechanical unreliability, error thresholds, and other expected deviations. Videos and flowcharts are provided to demonstrate the workflow of the device.

## Lab #12 Outcomes

For my inverted pendulum, I aimed to have the car start from a vertical orientation and add inputs into the system by driving the car forward. Although the drive forward was considered, it was ultimately ruled out considering the car extremely struggled to complete a flip. Generally, this uses the "YAW" measurement on the IMU to estimate instantaneous error, and drives both wheel pairs in the appropriate direction to offset the angular dispalcement.

This system uses a very simialr set-up to previous labs, with the helper functions created to store the PID values over BLE and PID inputs controlled/ processed in steps within the "void loop()" setting. The PID controller is a closed-loop with proportional, integral, and derivative gain at 20.0, 0.05, and 2.0 respectively. These were hand-tuned through trial-and-error to balance effective restoration and limited overshoot. The following are the basic code structures used in developing this inverted pendulum:

**CODE**

Additionally, a Kalman Filter is needed for this usage. Although the IMU used for testing has generally been reliable, the sample rate sometimes lags or is inconsistent which caused issues downstream in the workflow. This is additionally remedied by initializing and warming-up the IMU at the start of "INVERSE_PENDULUM" to ensure more consistent data acquisition at the start of the robot function. The following is the Kalman Filter integrated for this use case:

**CODE**

*Inverted Pendulum Operation*

<img src="https://ajd323.github.io/Fast-Robots-MAE4190/img/FR_Lab12_1.png" alt="Lab_12_1" style="max-width:700px; border-radius:12px; margin:0 0 0 0;" />

<img src="https://ajd323.github.io/Fast-Robots-MAE4190/img/FR_Lab12_2.png" alt="Lab_12_2" style="max-width:700px; border-radius:12px; margin:0 0 0 0;" />

<iframe width="560" height="315" src="https://www.youtube.com/embed/EvDm_t24ySM" title="Successful Inverted Pendulum" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

*Outcome Evaluation*

The robot appears to work moderately well for a short period of time. First, the PWM and PID error charts sent over from "BAL_TRANSMISSION" proved that the motor output was effective stablishing the system and reducing the pitch error. This is likely the effect of the Kalman Filter, which in addition to some imrpovement in IMU latency, prevented delays in balancing stepping in the main loop and prevented cascading error from building. By observing the packets transmitted through BLE, the robot has lower latency for processing with a 10 ms stepping rate saving at ~10.88 ms over a 10 second interval. Additionally, with other outlier removing code, the IMU provides high-accuracy data that prevents impacting long-term error. These trends are shown in the following graphs:

<img src="https://ajd323.github.io/Fast-Robots-MAE4190/img/FR_Lab12_3.png" alt="Lab_12_3" style="max-width:700px; border-radius:12px; margin:0 0 0 0;" />

<img src="https://ajd323.github.io/Fast-Robots-MAE4190/img/FR_Lab12_4.png" alt="Lab_12_4" style="max-width:700px; border-radius:12px; margin:0 0 0 0;" />

One of the major points of imporvemnt are the PID control gains choosen for the robot. From previous trials, the robot will sometimes drift contantly without resotring or fall immdeiately. There are multiple reasons for this issue. The foremost reasons for this is that the robot has an unconventional center-of-mass due to the addition of the 3D-printed plate. This means that the robot needs to be slightly angled (~85 degrees as oppsoed to 90 degrees) to maintain appropriate balance. Another reason is that the proportional and derivative gain may not be fully optimized, but have been tuned by hand as effectively as possible. While the sysetm restored moderately well, the car does aggressively readjust which sometimes causes large error. Additional trials were run to confirm that this combination of proportional and derivative were optimal. Changing the proportional gain prevented the car from effectively re-orienting and led to a perpetual drift. Changing the derivative gain resulted in agressive oscillations that resulted in higher amplitudes during settling. In the future, more mathematical approaches could be used to improve the outcomes. The following are some examples of "unsuccessful" trials during tuning:

*Failed Trial (With PID values of 7.5, 0.05, 2.0)*

<iframe width="560" height="315" src="https://www.youtube.com/embed/kqa6FrNtCM0" title="Unsuccessful Inverted Pendulum" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

*Failed Trial (With PID values of 20, 0.05, 1.5)*

<iframe width="560" height="315" src="https://www.youtube.com/embed/YTjb7QJMo4E" title="Unsuccessful Inverted Pendulum" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## Discussion



This lab was completed with Jamison Taylor, and assisted from AI tools for minor debugging. Thank you to all of the staff for your help throughout the semester, and I hope you all have a good summer!