# Encoder

This Library interfaces with the two Arduino Nano each connected with an Encoder. Also it resolves the readings into X and Y coordinates.

# Available Functions:

```
resetEncoder()                                 -           To Reset the Encoders
readEncoder()                                  -           To Read the current Encoder Values
encoderDebug()                               -           To print values of Encoder on change in values
getCoordinate(Coordinate &var)       -           Stores the X-Y Coordinates in var [Coordinate - Structure]
```

