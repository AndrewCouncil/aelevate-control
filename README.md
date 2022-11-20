# aelevate-control
ProjectIO Arduino code to control the Aelevate Bike Trainer for Duke Product Design

## Serial Interface Description

### Output
Every cycle, the Arduino will output over serial in the following format:

```
(unsigned long) current number of rotations (counted from hall effect sensor)
(unsigned long) current time, output from millis()
```

### Input
Every cycle, the Arduino will check for data input over serial of one of the following formats:

```
(char) 255
(char) 's'
```
In this case, it will reset the trainer to full resistance and lowest possible angle.

```
(char) 255
(char) 'p'
```
In this case, it will prompt the Arduino to begin writing position and time to serial.

```
(char) 255
(char) 'P'
```
In this case, it will prompt the Arduino to stop writing position and time to serial.

```
(char) 255
(char) 'r'
(unsigned char) desired resistance from 0 to 254
```
In this case, it will set the bike resistance to the value given, scaled so that 0 is the minimum possible resistance and 255 is the maximum possible resistance.

```
(char) 255
(char) 'a'
(unsigned char) desired angle from 0 to 254
```
In this case, it will set the bike angle to the value given, scaled so that 0 is the minimum possible angle and 255 is the maximum possible angle.

```
(char) 255
(char) 'd'
(unsigned char) desired resistance from 0 to 254
(unsigned char) desired angle from 0 to 254
```
In this case, it will set the bike resistance to the first value given, scaled so that 0 is the minimum possible resistance and 255 is the maximum possible resistance. It will also set the bike angle to the second value given, scaled so that 0 is the minimum possible angle and 255 is the maximum possible angle.
<!-- TODO: document track input if we actually use -->