# Kinematics Tool (Simple)

This folder contains a standalone tool for converting between Pose parameters and Joint angles.

## Files
- `kinematics_tool.cpp`: The source code containing IK and numerical FK logic.
- `build.bat`: A script to compile the tool using `g++`.
- `kinematics_tool.exe`: The compiled executable (appears after building).

## How to use
1. Run `build.bat` to compile the source code.
2. Run `kinematics_tool.exe`.
3. Follow the on-screen menu:
   - **Option 1 (IK)**: Input Pose (Yaw, Pitch, Z, Y, U5) to get Joint Angles (Theta 1-5).
   - **Option 2 (FK)**: Input Joint Angles (Theta 1-5) to get the resulting Pose.

## Requirements
- `g++` (MinGW) should be installed and in your system's PATH.
