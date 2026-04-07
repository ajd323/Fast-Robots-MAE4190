+++
title = "Lab 8: Stunts!"
date = 2026-04-07
description = ""
taxonomies = { tags = ["Closed PID", "Programmed Motion"] }
+++

## Summary Lab #8

Lab #8 is an opportunity to test the functionality of the linear/orientation PID controllers, the Kalman filters, and sensor sampling in a real-world example. With the architecture created from Lab #5 to Lab #7, a reliable flip and drift function is constructed. From this

## Lab #8 Outcomes

**Task A: Stunt Car Flip**

The flip functions has the following requirements to be considered successful from Lab #8:
- Start at a designated line <4 m away from the wall
- Perform a flip ~1 ft from the wall
- Drive back in the original direction

With this in mind, the following BLE case, "Flip," is designed to handle these requirements. Here is the major attribtues of the code packages in the Artemis Nano board.

```cpp
*CODE*
```

1. *Flip Test Success Trial #1*

*VIDEO 1*

*GRAPH 1*

2. *Flip Test Success Trial #2*

*VIDEO 2*

*GRAPH 2*

3. *Flip Test Success Trial #3*

*VIDEO 3*

*GRAPH 3*

**Task B: Stunt Car Drift**

The drift functions has the following requirements to be considered successful from Lab #8:
- Start at a designated line <4 m away from the wall
- Initiate a 180 degree turn when within 3ft of the wall

With this in mind, the following BLE case, "Drift," is designed to handle these requirements. Here is the major attribtues of the code packages in the Artemis Nano board.

```cpp
*CODE*
```

1. *Drift Test Success Trial #1*

*VIDEO 1*

*GRAPH 1*

2. *Drift Test Success Trial #2*

*VIDEO 2*

*GRAPH 2*

3. *Drift Test Success Trial #3*

*VIDEO 3*

*GRAPH 3*