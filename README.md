# NAO Robot Vision + Motion Control (NAOqi + OpenCV, C++) 
[Video Demo](https://youtu.be/r45bdkmdulo)<br>
This repository contains a C++ robotics project built for the SoftBank/Aldebaran NAO humanoid robot using NAOqi proxies and OpenCV. The system integrates vision-based perception (red ball + landmark detection) with motion planning/control (walking, head scanning, alignment, and “kick/hit” actions) as a state-driven autonomous behavior.
## What's inside
### Repository structure
```graphql
nao-robot-vision-motion/
├─ README.md
├─ LICENSE
├─ NOTICE
├─ .gitignore
├─ CMakeLists.txt
├─ src/
│  ├─ nao_main.cpp
│  └─ nao_main_alt_ip.cpp
├─ assets/
│  ├─ diagrams/
│  │  └─ nao_vision_motion_pipeline.svg
│  └─ media/
│     ├─ NAO.jpg
│     ├─ Nao_Robot_(Robocup_2016).jpg
│     └─ STQZ2432.MP4
└─ docs/
   ├─ slides/
   │  └─ Motion Control and Computer Vision Application in autonomous.pptx
   └─ notes/
      ├─ CompVisi_Chloe.txt
      ├─ modified-red-ball-det.txt
      ├─ red-ball-det.txt
      └─ vision-exp.txt
```

### Core C++ programs (NAOqi + OpenCV)
* [src/nao_main.cpp](src/nao_main.cpp) <br>
  Main integrated behavior: connects to NAO via NAOqi proxies, scans with head motion, detects targets (red ball/landmarks), estimates relative pose, and executes walking + alignment + action logic.
* [src/nao_main_alt_ip.cpp](src/nao_main_alt_ip.cpp) <br>
A second variant of the same project (e.g., a different robot competition iteration).
### Documentation & competition materials
* [docs/slides/](docs/slides/) — presentation decks
* [docs/notes/](docs/Notes/) — code snippets

## System Pipeline

![NAO Vision–Motion Pipeline](assets/diagrams/nao_vision_motion_pipeline.svg)

## System architecture (high-level)
1. Perception
* Camera stream from NAO (ALVideoDevice)
* Detection primitives:
  * Red ball detection (NAOqi +/or OpenCV pipeline)
  * Landmark detection (ALLandMarkDetection)
* Optional orientation aid: Visual compass (ALVisualCompass)
2. Decision / State machine
* States such as:
  * SEARCH_RED_BALL
  * FIND_LANDMARK
  * alignment/approach/verify
  * action execution (kick/hit)
3. Motion & posture control
* ALMotionProxy for walking and joint control
* ALRobotPostureProxy for posture initialization
* Configurable gait parameters
## Dependencies
* NAOqi C++ SDK (Aldebaran/SoftBank Robotics)
* OpenCV (headers and libs available to your toolchain)
* A Linux environment matching your NAOqi toolchain (often required for C++ builds)
## Configure robot connection
In the source, the robot IP is hardcoded (e.g., robotIP "192.168.1.101" / "192.168.1.103").
Update it to your robot’s IP before building/running.
## Build & Run (typical workflow)
Exact build steps depend on how your NAOqi SDK is installed (qiBuild vs manual toolchain).
This repo is structured so you can adapt either approach cleanly.
### Option A — qiBuild (common NAOqi workflow)
1. Install/configure qiBuild + NAOqi SDK
2. Create a build folder and compile:
```bash
qibuild configure
qibuild make
```
3. Deploy binary to NAO (or run remotely depending on your setup)
### Option B — manual CMake/g++ (if your SDK exposes include/lib paths)
You’ll need to link against NAOqi libraries and OpenCV manually using your SDK’s include/lib directories.
## Notes on licensing/attribution
Some files include headers from Aldebaran Robotics example code.
See [NOTICE](NOTICE) for attribution and keep original headers intact.



