+++
title = "Lab 1: Robot Startup"
date = 2026-01-30
description = "Getting the robot up and running with basic motor control."
taxonomies = { tags = ["embedded", "motors"] }
+++

## Objectives

- Set up the microcontroller development environment
- Implement PWM for motor speed control
- Create a simple testing framework
- Establish serial communication for debugging

## Key Learnings

The importance of proper power management and noise filtering in motor control circuits became immediately apparent. We discovered that without adequate decoupling capacitors, the PWM noise was interfering with sensor readings.

## Results

Successfully achieved consistent motor speed control across a 0-255 PWM range with less than 5% deviation.