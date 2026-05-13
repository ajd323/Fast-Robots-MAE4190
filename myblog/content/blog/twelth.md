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

This system uses a very simialr set-up to previous labs, with the helper functions created to store the PID values over BLE and PID inputs controlled/ processed in steps within the "void loop()" setting. The PID controller is a closed-loop with proportional, integral, and derivative gain at ___, __, and respectively. These were hand-tuned through trial-and-error to balance effective restoration and limited overshoot. The following are the basic code structures used in developing this inverted pendulum:

**CODE**

Additionally, a Kalman Filter is needed for this usage. Although the IMU used for testing has generally been reliable, the sample rate sometimes lags or is inconsistent which caused issues downstream in the workflow. This is additionally remedied by initializing and warming-up the IMU at the start of "INVERSE_PENDULUM" to ensure more consistent data acquisition at the start of the robot function. The following is the Kalman Filter integrated for this use case:

**CODE**

*Inverted Pendulum Operation*

**DIAGRAM OF OPERATION**

**VIDEO OF OPERATION**

*Outcome Evaluation*

The robot appears to work moderately well for a short period of time. By observing the packets transmitted through BLE, the robot has lower latency for processing with a 20 ms stepping rate saving at ~21.5 ms over a 10 second interval. Additionally, the graphs provided are smooth due to the Kalman Filter and no outliers are propogating into the data impacting motor control. This is shown in the following graph:

**PHOTO OF MOTOR CONtROL**

One of the major points of imporvemnt are the PID control gains choosen for the robot. From previous trials, the robot will sometimers drift contantly without resotring or fall immdeiately. These issues are related to balancing the proportional and derivative gain, and although these have been tuned by hand as effectively as possible, more mathematical approaches may imporve the outcomes. Here is an example of an "unsuccessful" trial:

**VIDEO OF OPERATION**

## Discussion



This lab was completed with Jamison Taylor, and assisted from AI tools for minor debugging. Thank you to all of the staff for your help throughout the semester, and I hope you all have a good summer!