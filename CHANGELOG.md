# ThorCore 0.5.1

Added Seeker - a NEONVision-based object tracker. In testing, it runs fast enough that the limiting 
factor of pipeline speed is the camera framerate.

Added PowerPlaySeeker - a variant of the Seeker tuned specifically for the 2022-2023 Power Play season.
It is capable of tracking blue cones & tape, red cones & tape, and poles.

Added SamplePowerPlayVision - a sample that shows how to use PowerPlaySeeker.

Added the Hardware Debugger - a tool that displays information about sensors, motors, encoders, and
other robot hardware over telemetry. Useful for troubleshooting wiring, configuration, and electrical
issues.

# ThorCore 0.5.0

Updated to FTC SDK 8.1

# ThorCore 0.4.1

Added mask support to NEONVision
Added NativeImageByteBuffer
Added SampleNEONVisionMask
Completed update to SDK 7.0