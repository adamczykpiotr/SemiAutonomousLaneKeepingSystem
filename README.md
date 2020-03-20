# Semi-autonomous lane keeping system
Semi-autonomous lane keeping system based on C++ & OpenCV 4.x.\
Project for my engineering thesis at AGH University of Science and Technology [WIMiIP].\
\
![](https://github.com/adamczykpiotr/SemiAutonomousLaneKeepingSystem/blob/master/preview.gif)


## Building
Linux:   `./build.sh`\
Windows: Using Visual Studio 2019


## Depencencies
Linux: `./build.sh install`\
Windows: Download OpenCV in version 4.1.1 and extract it to `C:\opencv` then copy all files from `C:\opencv\build\x64\vc15\bin` to `VisualStudio\x64\Debug` and `Release` directories

##  Progress history
| Task & Commit | Date |
| ---------- | ----------- |
| [Finished readme](https://github.com/adamczykpiotr/SemiAutonomousLaneKeepingSystem/commit/bf5befb4afbd0a68cf6ef7d568b5c7bef2e1be17) | 2020-03-18 |
| [Build tools](https://github.com/adamczykpiotr/SemiAutonomousLaneKeepingSystem/commit/5bde61885038f58fb816b5f0e4133f4470bd098c) | 2020-03-20 |
| [Lane detection](https://github.com/adamczykpiotr/SemiAutonomousLaneKeepingSystem/commit/b1bad0931d3f59c01665718d5ac228b2ae21bc87) | 2020-03-20 |



## To do
- [ ] Software
    - [x] Build tools
    - [x] Road lane recognition
        - [ ] Turn prediction
        - [ ] Histheresis
        - [ ] Optimization
            - [ ] Multi-threading
            - [ ] ARM NEON optimizations
        - [ ] PID controller for steering angle calculation
- [ ] Hardware
    - [ ] Opel EPS-4400 reverse engineering
        - [x] Gather all necessary datasheets
        - [ ] Board schematics based on PCB
        - [ ] Inject own steering signals
        - [ ] Prevent triggering DTCs (Diagnostic Trouble Codes)
    - [ ] Raspberry Pi 4 <-> Power steering
        - [ ] Communication
- [ ] Safety measures
    - [ ] Software
    - [ ] Hardware

## References 
**General concept**\
https://medium.com/@mrhwick/simple-lane-detection-with-opencv-bfeb6ae54ec0\
https://medium.com/pharos-production/road-lane-recognition-with-opencv-and-ios-a892a3ab635c)\

**Use of Gaussian blur & simple turn prediction**\
https://github.com/MichiMaestre/Lane-Detection-for-Autonomous-Cars

**PID Controller**\
https://en.wikipedia.org/wiki/PID_controller\
https://www.youtube.com/watch?v=4Y7zG48uHRo


