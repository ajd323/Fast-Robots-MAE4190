+++
title = "Lab 2: Sensor Integration"
date = 2026-02-05
description = "Integrating IMU and distance sensors for autonomous navigation."
taxonomies = { tags = ["sensors", "imu", "calibration"] }
+++

## Objectives

- Calibrate and integrate the IMU (gyroscope, accelerometer)
- Read distance sensors accurately
- Implement sensor fusion algorithms
- Develop autonomous obstacle detection

## Challenges & Solutions

The IMU required careful calibration due to temperature sensitivity. We implemented a warm-up period and temperature compensation algorithm that reduced drift significantly.

## Performance Metrics

- IMU accuracy: ±2 degrees
- Distance sensor range: 5-200 cm
- Update rate: 50 Hz