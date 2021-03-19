# ThorCore
ThorCore is a library for FTC. It implements common drivetrains, PIDF controllers, 2D Vector math,
and more. Additionally, it provides navigation systems and development tools.

This is the dev branch. Code here may be unfinished or operating incorrectly.


#### Installation
To install, clone ThorCore into your Android Studio project. The ThorCore folder should be in the
top-level directory, alongside ```FtcRobotController``` and ```TeamCode```.
For example, if the project was in ```/home/thor/ftc/FtcRobotController```:
```bash
thor@thor:~$ cd /home/thor/ftc/FtcRobotController
thor@thor:~/ftc/FtcRobotController$ git clone -b dev https://github.com/FTC-9974-THOR/ThorCore.git
thor@thor:~/ftc/FtcRobotController$ cd ThorCore
thor@thor:~/ftc/FtcRobotController/ThorCore$ git submodule update --init --recursive
```
Start up Android Studio and open the project. In ```settings.gradle```, add the following line at the
start of the file:
```gradle
include ':ThorCore'
```
In ```build.common.gradle```, change ```minSdkVersion``` and the Java compile options:
```gradle
...
android {
    ...
    defaultConfig {
        ...
        minSdkVersion 25
        ...
    }

    ...

    compileOptions {
        sourceCompatibility JavaVersion.VERSION_1_8
        targetCompatibility JavaVersion.VERSION_1_8
    }

    ...
}
...
```
Last, you need to include ThorCore as a dependency. Open
```TeamCode/build.gradle``` and put the following code at the end:
```gradle
dependencies {
    implementation project(':ThorCore')
}

android {
    compileOptions {
        sourceCompatibility JavaVersion.VERSION_1_8
        targetCompatibility JavaVersion.VERSION_1_8
    }
 }
 ```

Run a Gradle sync, and you should be all set. Android Studio should automatically download and install
the NDK and CMake, but you can download them manually from the SDK Manager if necessary.

### Note: the rest of this document is outdated. It needs to be updated to reflect SDK changes.

#### Features
* PIDs
* The [Realizer](https://github.com/FTC-9974-THOR/ThorCore/tree/master/src/main/java/org/ftc9974/thorcore/meta/Realizer.java) -
  a clean, reflection-based way of initializing hardware.
* Pre-made implementations of drivetrains
* The [Navigator](https://github.com/FTC-9974-THOR/ThorCore/tree/master/src/main/java/org/ftc9974/thorcore/control/Navigator.java) -
  autonomous navigation with field-relative coordinates
* The [TFDetector](https://github.com/FTC-9974-THOR/ThorCore/tree/master/src/main/java/org/ftc9974/thorcore/control/TFDetector.java) -
  Tensorflow, packaged into a neat little class
* The Telepathy API - a more powerful form of telemetry, with an [open source client](https://github.com/FTC-9974-THOR/TelepathyClient) that runs on a computer

#### Documentation
Primary documentation is in-code Javadocs. There is also sample code
available [here](https://github.com/FTC-9974-THOR/ThorCore/tree/master/src/main/java/org/ftc9974/thorcore/samples), with more samples coming soon.

***
For additional information, submit an issue or DM @fortraan#2768 on Discord (note that you'll have to be in the FTC Discord server to do so).
***
*ThorCore v0.3*