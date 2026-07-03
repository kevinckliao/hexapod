# Minimal Hexapod for MuJoCo 3.10.0
Open scene.xml in MuJoCo Studio.

# Versions:
[Python:] 3.11
[Packages:]
    PyOpenGL==3.1.10
    PyYAML==6.0.3
    etils==1.14.0
    mujoco==3.10.0
    pip-chill==1.0.5
    xacro==2.1.1

# Install Procedure
py -3.11 -m venv venvMujoco
venvMujoco\Scripts\activate
python -m pip --upgrade pip
pip install -r requirements.txt 

# Visual Studio Code (vsc) setup
Ctrl+Shift+P
Python: Select Intepreter
    --> venvMujoco\python.exe
In Python Source-Code, Click [Play]-Triangle-Icon

# Description:
This is a simple educational template:
- 6 legs
- 18 hinge joints
- Position actuators
- Ground plane
- Capsule links

Use it as a starting point. Replace capsules with meshes later.
"# hexapod" 
