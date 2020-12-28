# Note: This readme is out of date and is under revision.

# ThorCore
ThorCore is a library for FTC.

This branch is for stable releases. The dev branch contains newer code, but not all of it may work.

#### Installation
To install, start up a terminal.
```
thor@thor:~$
```
Clone the ThorCore repo.
```
thor@thor:~$ git clone https://github.com/FTC-9974-THOR/ThorCore
```
Start up Android Studio. Select ```File > New > Import module...``` and
enter the directory you cloned ThorCore to. If everything worked right,
you should be able to run a Gradle sync without errors.

Last, you need to include ThorCore as a dependency. Open
TeamCode/build.gradle and put the following code at the end:
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
Run a Gradle sync, and you should be all set.

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
For additional information, submit an issue or pm /u/ardx_zero on Reddit.
***
*ThorCore v0.2*
