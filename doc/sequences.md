# Sequences

The sequences performed during the joint lift operation can be specified through the use of a configuration file. The default configuration file is located in the config directory as `sequences.yaml`.

There are currently three different types of sequences that can be executed: lines, triangles and circles. 

## Example Config

Here is an example of a sequence configuration

``` yaml
---
sequences:
  - type: "line"
    dx: 0
    dy: 0
    dz: 0.1

  - type: "line"
    dx: 0
    dy: 0
    dz: 0.1

  - type: "line"
    dx: 0
    dy: 0.225
    dz: 0.05

  - type: "circle"
    radius: 0.08
    theta_i: 0
    direction: "ccw"
    revolutions: 5

  - type: "line"
    dx: 0
    dy: -0.225
    dz: -0.05

  - type: "line"
    dx: 0
    dy: 0
    dz: -0.2
```

## Line sequence

The robot will move in a straight line from the current position to a point defined by the change in each three directions in the world coordinate system.

| Parameter | Type   | Description | 
| --------- | ------ | ----------- |
| **type**  | string | type of sequence ("line") |
| **dx**    | number | amount of movement in x direction in meters |
| **dy**    | number | amount of movement in y direction in meters |
| **dz**    | number | amount of movement in z direction in meters |

## Triangle sequence

The robot will move in a equilateral triangle in the Y-Z plane

| Parameter       | Type   | Description | 
| --------------- | ------ | ----------- |
| **type**        | string | type of sequence ("triangle") |
| **side_length** | string | length of one side of triangle in meters |
| **direction**   | string | direction of triangle ('cw' or 'ccw') |
| **revolutions** | number | number of revolutions |

## Circle sequence

The robot will move in a circle in the Y-Z plane

| Parameter       | Type   | Description | 
| --------------- | ------ | ----------- |
| **type**        | string | type of sequence ("circle") |
| **radius**      | number | radius of circle in meters |
| **theta_i**     | number | initial angle of circle in degrees (0 is top of circle) |
| **direction**   | string | direction of triangle ('cw' or 'ccw') |
| **revolutions** | number | number of revolutions |