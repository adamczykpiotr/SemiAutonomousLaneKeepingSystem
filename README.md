# Semi-autonomous lane keeping system
Semi-autonomous lane keeping system based on C++ & OpenCV 4.x.\
Project for my engineering thesis at AGH University of Science and Technology [WIMiIP].


## Building
Linux:   build_linux.sh    - builds whole app\
Windows: build_windows.cmd - creates Visual Studio 2019 project

any required libs will be automatically downloaded & installed in correct versions for respective platforms

##  Progress history
| Task & Commit | Date |
| ---------- | ----------- |
| [Finished readme](https://github.com/adamczykpiotr/SemiAutonomousLaneKeepingSystem/commit/bf5befb4afbd0a68cf6ef7d568b5c7bef2e1be17) | 2020-03-18 |

## To do
- [ ] Software
    - [ ] Build tools
    - [ ] Road lane recognition
        - [ ] Turn prediction
        - [ ] Histheresis
        - [ ] Optimization
            - [ ] Multi-threading
            - [ ] ARM NEON optimizations?
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


