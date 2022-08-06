# RobotCode

## Coding Resources

[Crc Lib](https://robocrc.atlassian.net/wiki/spaces/AR/pages/637567103/English+Section+-+Intro+Page)

## What we've learned

1. 5V rails don't work unless 12V is supplied to board.
2. Can't program the board if wireless communication is active.


## Todo

- Implement a clock/time management (if needed)
- Find a way to separate the code into multiple files

## Requires Testing

- Drive train movement, speed, etc.

## Done

- Controller library
- Drive mode switching


## Controls

### Global

- Y button: Switch drive (toggle)
- X button: Output chain (hold)
- A button: Activate input (toggle)
- Joystick 2, y: Slide angle
  - if up: up
  - if down: down
  - Constant speed
- D-pad (up/down): Slide presets (full effect to be determined)

### Tank Drive

- Triggers: Acceleration, Deceleration (Analog)
- Joystick 1, X axis: Turning (Analog)
- B: Boost (hold) -> unlock motor speed
  - note: may not be necessary, will be determined when drive train is tested in drive mode.

### Omni Drive

- Significantly slower
- Joystick 1: direction + speed
- bumbers: rotation
