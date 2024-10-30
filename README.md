### Masters_Thesis_SDU

## Setup Windows

# Code
1. Setup python environment
```
C:\Users\Admin\AppData\Local\Programs\Python\Python39\python.exe -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

# VR Headset (Meta Quest)
2. Install and setup meta link app
https://www.oculus.com/download_app/?id=1582076955407037

3. Install SteamVR
https://store.steampowered.com/app/250820/SteamVR/

4. Launch SteamVR. make sure headset is connected and working

# Turns out latest bullet version 3.25 works with VR but crashes when loading softbody. Use these steps if you don't need soft body, otherwise look bellow 
5. Download bullet3 source
https://github.com/bulletphysics/bullet3/releases/tag/3.25

6. Unpack it. Open "build_visual_studio_vr_pybullet_double.bat" with notepad and edit the script so it finds your python installation folder

7. run build_visual_studio_vr_pybullet_double.bat

8. Open bullet3-3.25\build3\vs2010\0_Bullet3Solution.sln with visual studio

9. Retarget solution so it supports newer visual studio that you are using

10. in visual studio > build > configuration manager, select release version

11. build App_PhysicsServer_SharedMemory_VR

12. Run VR server bullet3-3.25\bin\App_PhysicsServer_SharedMemory_VR_vs2010_x64_release.exe before running pubyllet with shared_memory client (VR only)

# VR with SoftBody support

5. Git clone bullet3 master, which is currently at version 3.26

git clone git@github.com:bulletphysics/bullet3.git

6. Building with premake is outdated and softbody doesnt work. run build_visual_studio_vr_pybullet_double_cmake.bat instead. This needs some preparation

6.1 Download Cmake https://cmake.org/download/

6.2 modify the bat file to point to corrent visual studio version. I used -G "Visual Studio 17 2022"

6.3 modify bat file to point to correct python locations

6.4 add -DUSE_OPENVR=ON flag to cmake flags

7. open bullet3\build_cmake\BULLET_PHYSICS.sln with visual studio and build App_PhysicsServer_SharedMemory_VR

8. The instalation places .dll file in the wrong directory. Move bullet3\build_cmake\examples\SharedMemory\openvr64pi.dll to bullet3\build_cmake\examples\SharedMemory\Debug\openvr64pi.dll

9. Start VR server bullet3\build_cmake\examples\SharedMemory\Debug\App_PhysicsServer_SharedMemory_VR



# Linux (depricated, poor VR support)
```
python3.9 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```