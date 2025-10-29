# Robotic project (Autonomous Mobile Manipulator)
An autonomous robot capable of detecting, grasping, transporting, and placing objects within a 1.5m square arena.

## Overview
This project features a two-wheeled mobile base integrated with a single-degree-of-freedom arm and motorized gripper. The robot operates fully autonomously using a finite state machine architecture that coordinates navigation, object detection, manipulation, and placement sequences.

## Key Features
- **State machine control**: Behavior sequencing triggered by sensor input and beacon detection
- **Autonomous manipulation**: Complete pick-and-place pipeline from detection to final placement
- **Custom mechanical design**: Laser-cut and 3D-printed components fabricated in FABLAB
- **Embedded systems**: Arduino-based control with ultrasonic sensors and motor drivers

## Technical Specifications
- Two-wheeled differential drive mobile base
- Single-DOF articulated arm with motorized gripper
- Ultrasonic sensors for obstacle detection and object localization
- Visual beacon recognition for state transitions
- 12V battery power system
- Arduino microcontroller

## System Architecture
The robot implements a finite state machine with the following states:
- Navigation and arena exploration
- Object detection and localization
- Approach and positioning
- Grasping
- Transport
- Placement and release

> [!WARNING] Warning
> Please look at the different branches to find the StateMachine code.
