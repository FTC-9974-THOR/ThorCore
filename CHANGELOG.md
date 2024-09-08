# ThorCore 1.1.0

Updated to FTC SDK v10.0

# ThorCore 1.0.2

Refactored feedforward calculation in SwerveModule2

Further trimmed old and obsolete code

# ThorCore 1.0.1

Updated to FTC SDK v9.2

# ThorCore 1.0.0

Added a new RoadRunner-compatible navigation system. This system includes support for both
pure-odometry localization and a sensor fusion localizer that fuses readings from odometry, the IMU,
and AprilTags.

Added code for modern swerve drives. ThorCore already had code for swerve drives, but it was written
during the very early days of swerve in FTC. Now that swerves have become more commonplace, it was
time for an update. The new code supports swerve drives that use a motor for driving and an Axon
servo for turning. It also uses a much simpler, more flexible set of swerve kinematics and includes
turn-key support for RoadRunner via the new navigation system.

Added PIDFv - a PIDF controller that supports both position and velocity setpoints.

Added SlewRateLimiter - a tool that limits the slew rate of a signal.

Added AutoHeadingHold - a driver-assist tool designed for countering heading drift during teleop.

Added very basic support for complex arithmetic.

Added a few circular statistics methods to MathUtilities.

Removed final qualification from classes.

Removed:
 * TFDetector
 * VIPD
 * VPIDF
 * Matrix
 * OldMatrix
 * Tensor
 * CompositeNavSource
 * ProfiledMovementStrategy
 * SensorFusionNavStrategy
 * VuMarkNavSource
 * StandaloneStackVision

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