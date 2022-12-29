# ThorCore
ThorCore is a library for FTC. It provides implementations for common drivetrains, vector mathematics,
closed-loop controllers, vision systems, and sensors.

This is the dev branch. Some of the code here may be unfinished or unstable, but most things should work.

### Installation
There are 2 ways to install ThorCore.
#### Packaged Binary
Download the latest .aar file from the Releases page. Copy it into the ```libs``` folder in your
robot controller project. Next, open the ```build.gradle``` file in the ```TeamCode``` module. Inside
the dependencies block, add the following line:
```gradle
implementation name: "ThorCore", version: "0.5+", ext: "aar"
```
Run a Gradle sync, and it should be all set.
#### From Source
Clone ThorCore into your Android Studio project. The ThorCore folder should be in the
top-level directory, alongside ```FtcRobotController``` and ```TeamCode```. Make sure you include
git submodules in the cloning process.
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
In ```TeamCode/build.gradle```, insert the following at the end of the file:
```gradle
android {
    defaultConfig.minSdkVersion 25
    compileOptions.sourceCompatibility JavaVersion.VERSION_1_8
    compileOptions.targetCompatibility JavaVersion.VERSION_1_8
}

dependencies {
    implementation project(':ThorCore')
}
```

Run a Gradle sync, and you should be all set. Android Studio should automatically download and install
the NDK and CMake, but you can download them manually from the SDK Manager if necessary. ThorCore is
built and tested with NDK version 25.1.8937393 and CMake 3.18.1, but it should work with any recent
version of the NDK and CMake.

***
For additional information, submit an issue or DM @fortraan#2768 on Discord (note that you'll have to be in the FTC Discord server to do so).
***
*ThorCore v0.5.0*