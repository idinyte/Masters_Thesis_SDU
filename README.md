### Masters_Thesis_SDU

## Setup Windows

# Code
1. Setup python environment
```
C:\Users\<YourUser>\AppData\Local\Programs\Python\Python39\python.exe -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

# VR Headset (Meta Quest)
2. Install and setup meta link app
https://www.oculus.com/download_app/?id=1582076955407037

3. Install SteamVR
https://store.steampowered.com/app/250820/SteamVR/

4. Launch SteamVR. make sure headset is connected and working

5. Download bullet3 source
https://github.com/bulletphysics/bullet3/releases/tag/3.25

6. Unpack it. Open "build_visual_studio_vr_pybullet_double.bat" with notepad and edit the script so it finds your python installation folder

7. run build_visual_studio_vr_pybullet_double.bat

8. Open bullet3-3.25\build3\vs2010\0_Bullet3Solution.sln with visual studio

9. Retarget solution so it supports newer visual studio that you are using

10. in visual studio > build > configuration manager, select release version

11. build App_PhysicsServer_SharedMemory_VR

12. Run VR server bullet3-3.25\bin\App_PhysicsServer_SharedMemory_VR_vs2010_x64_release.exe before running pubyllet with shared_memory client (VR only)

# Linux (depricated, poor VR support)
```
python3.9 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```