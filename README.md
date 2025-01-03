# VM_350
This is UM_SJTU JI 2024SU VM350 final projects. 

If there are similar problems or projects in the future, JI students are responsible for not copying or modifying these codes or files, as this would be against the Honor Code.

The owner of this repository should not be held responsible for anyone else's errors.

# Autonomous Vehicle Control Code for Game Zone Task

## Overview

This Python program controls an autonomous vehicle designed to complete a series of tasks in a simulated exploration game zone. The vehicle navigates complex terrains, makes decisions based on sensor data and camera input, and achieves the objectives outlined in the task rules.

The tasks include:
- Starting and navigating through obstacles (steps, sandboxes).
- Adjusting height to pass under barriers.
- Identifying turning directions based on color-coded signs.
- Parking in a designated area.

This program integrates GPIO control for motors and sensors, image processing for decision-making, and movement logic to finish the challenge autonomously.

## Features

### Motor Control
- Forward, backward, left-turn, and right-turn movement.
- Adjustable speed and duration for precise motion control.

### Obstacle Navigation
- Sensor-based distance detection to traverse terrains like steps and sandboxes.
- Automated adjustments using transformable wheels.

### Camera Vision
- Detects directional signs using HSV-based color filtering.
- Recognizes red and green contours to decide the vehicle's turning direction.

### Height Adjustment
- Opens and closes wheels to pass under a height restriction bar.

### Parking
- Stops the vehicle in a predefined parking area after crossing the finish line.

## Prerequisites

### Hardware
- Raspberry Pi (with GPIO pins).
- Motors and motor drivers for left and right wheels.
- Distance sensors (e.g., ultrasonic sensors) for obstacle detection.
- Servo motors for wheel transformations.
- Camera Module for directional detection.

### Software
- Python 3.x.
- Libraries: RPi.GPIO, time, cv2 (OpenCV), numpy, Picamera2.

## Code Structure

1. **Initialization**
   - GPIO pins for motors, sensors, and servos are defined and set up.
   - Sensors are initialized for obstacle detection.
   - The `Car` class handles all movement, sensor reading, and wheel transformation logic.

2. **Main Loop**
   - The program continuously checks for a start signal via a button press.
   - Tasks are performed sequentially based on the game rules:
     - Forward Movement: Starts moving forward until an obstacle is detected.
     - Obstacle Handling: Uses distance sensors to climb steps and traverse sandboxes.
     - Wheel Transformation: Adjusts height to pass under a restriction bar.
     - Turning Decision: Camera detects green and red signs, and the vehicle turns accordingly.
     - Parking: Stops in the parking area after completing the course.

3. **Camera Processing**
   - Captures real-time video frames using the Pi camera.
   - Filters colors (HSV) to detect green and red contours.
   - Decides the direction (left or right) based on contour positions.

4. **Error Handling**
   - Includes a keyboard interrupt handler to safely clean up GPIO settings on exit.

## How to Use

### Setup Hardware
1. Connect motors, sensors, servo motors, and the camera module to the Raspberry Pi as per the pin configuration in the code.

### Run the Program
1. Save the script as `main.py`.
2. Execute the script using:
    ```sh
    python3 main.py
    ```
3. Press the button to start the vehicle.

### Monitor Logs
- The terminal displays logs of the vehicle’s actions, sensor readings, and camera detections.

### Stop the Program
- Press `Ctrl + C` to stop the program and clean up GPIO settings.

## Key Components

### Motor Control
- Motors are controlled via GPIO pins using `Car.forward()`, `Car.backward()`, `Car.turn_left()`, and `Car.turn_right()` methods.

### Sensors
- Ultrasonic sensors measure distances to detect obstacles.
- Logic adjusts vehicle movement based on sensor feedback.

### Camera Processing
- HSV filtering identifies the color-coded signs for turning directions.
- Contour detection determines the relative positions of red and green areas.

### Wheel Transformation
- `Car.Wheels_open()` and `Car.Wheels_closed()` adjust the vehicle’s height.

## Troubleshooting

### No Movement
- Verify motor connections and GPIO pin assignments.
- Check for sufficient power supply to motors and servos.

### Incorrect Turning
- Adjust HSV values in the camera section to better detect red and green signs.

### Sensor Malfunction
- Ensure sensors are correctly connected and oriented.
- Test sensor readings independently using a simple script.

### Camera Issues
- Ensure the camera is properly connected and enabled in Raspberry Pi settings.

## Future Improvements

- Optimize the camera detection algorithm for faster frame processing.
- Add error correction to handle unexpected obstacles.
- Implement additional sensors for enhanced terrain adaptability.
```
