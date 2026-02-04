# NAO Robot Vision + Motion Control (NAOqi + OpenCV, C++) 
This repository contains a C++ robotics project built for the SoftBank/Aldebaran NAO humanoid robot using NAOqi proxies and OpenCV. The system integrates vision-based perception (red ball + landmark detection) with motion planning/control (walking, head scanning, alignment, and “kick/hit” actions) as a state-driven autonomous behavior.
## What's inside
### Core C++ programs (NAOqi + OpenCV)
* [src/nao_main.cpp](src/nao_main.cpp) <br>
  Main integrated behavior: connects to NAO via NAOqi proxies, scans with head motion, detects targets (red ball/landmarks), estimates relative pose, and executes walking + alignment + action logic.
* [src/nao_main_alt_ip.cpp](src/nao_main_alt_ip.cpp) <br>
A second variant of the same project (e.g., a different robot competition iteration).
### Documentation & competition materials
* [docs/reports/](docs/reports) — PDF writeups (OpenCV + NAO-related references)
* [docs/slides/](docs/slides/) — presentation decks
* [docs/notes/](docs/notes/) — text notes / code snippets
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
'''
