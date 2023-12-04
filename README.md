# D.A.S.O.M
__Development of Aerial Manipulator System for Object Manipulation__
<br/>
<br/>
<img align="left" src="https://github.com/S-CHOI-S/D.A.S.O.M/assets/113012648/1bbd5748-1b4b-4474-8929-b9e2390c47ed" width="50%" height="50%"/>  
<br/>
Project period: 2023.01. ~ 2024.01.  

<br/><br/><br/><br/>
> [!note]
> _Contributions!:_ 
> _Admittance Control, External Force Estimation, Teleoperation, Disturbance Observer, Gimbaling_

> Our experiment video can be found at: https://youtu.be/yVvbRuphMrc?si=mRRZsSERV7XZkeGD

<br/>

## ICROS 2023: 2023.06.21 ~ 2023.06.23 
Manipulator System Based on Compliance Control For Object Manipulation  
[![Typing SVG](https://readme-typing-svg.demolab.com?font=Roboto&size=15&pause=1000&color=F72213&width=435&lines=%5BBest+Paper+Award-Winner%5D)](https://git.io/typing-svg)  
<img align="left" src="https://github.com/S-CHOI-S/S-CHOI-S/assets/113012648/2e1e9a4b-2c28-4470-80d0-89f2a0bbe4c2" width="50%" height="50%"/>  
<br/><br/><br/><br/><br/><br/><br/><br/>  
(You can check the publication in this page!: <a href="https://www.notion.so/pineasol/Publications-fc1044dd280544079cae4b1204109b53?pvs=4"><img src="https://img.shields.io/badge/Publications-FFFFFF?style=flat-square&logo=Notion&logoColor=black&link=https://www.notion.so/pineasol/Publications-fc1044dd280544079cae4b1204109b53?pvs=4"/>)

<br/>

## Capstone Design 2023: 2023.02.20 ~ 2023.11.24
Development of Aerial Manipulator System for Object Manipulation  
[![Typing SVG](https://readme-typing-svg.demolab.com?font=Roboto&size=15&pause=1000&color=F72213&width=435&lines=%5BCapstone+Design+1st+Place+Winner%5D)](https://git.io/typing-svg)  
(You can check the publication in this page!: <a href="https://www.notion.so/pineasol/Publications-fc1044dd280544079cae4b1204109b53?pvs=4"><img src="https://img.shields.io/badge/Publications-FFFFFF?style=flat-square&logo=Notion&logoColor=black&link=https://www.notion.so/pineasol/Publications-fc1044dd280544079cae4b1204109b53?pvs=4"/>)

<br/>

### Control Diagram
<img align="left" src="https://github.com/S-CHOI-S/D.A.S.O.M/assets/113012648/6d163de3-d29f-4e70-a752-75571233e692" width="70%" height="70%"/>
<br/><br/><br/><br/><br/><br/><br/><br/><br/><br/><br/><br/> 

### Interface (GUI)
> [!Tip]
> For User (in control center): can manipulate objects while looking at the screen of the GUI

__Default__  
  <img align="center" src="https://github.com/S-CHOI-S/D.A.S.O.M/assets/113012648/fbde13d5-d6be-40e8-9640-d21d4c874222" width="70%" height="70%"/>

__Running__  
  <img align="left" src="https://github.com/S-CHOI-S/D.A.S.O.M/assets/113012648/bca287cf-c504-4f3c-95ae-a04f5ddfaa3c" width="70%" height="70%"/>  

<br/><br/><br/><br/><br/><br/><br/><br/><br/><br/><br/><br/><br/><br/><br/><br/> 

## Installation
### Hardware
__Essential 1.__ OptiTrack
> OptiTrack is a Motion Capture System and in this study, we used ROS package made by: [MIT Aerospace Controls Laboratory](https://github.com/mit-acl)
```shell
https://github.com/mit-acl/optitrack.git
```

__Essential 2.__ 3D Geomagic Touch - Haptic Device  

> 3D Geomagic Touch allows the user to feel the virtual object and creates a real sense.  
> We modified ROS package made by: [fsuarez6](https://github.com/fsuarez6)

```shell
https://github.com/fsuarez6/phantom_omni.git
```
>[!Caution]
> Need to install _geomagic_touch_device_driver_ !

<br/>

### Software
__Essential 1.__ Install Dependencies
> Dynamixel Workbench 1.0.0 version required

>[!Tip]
> Installation guide: [ROBOTIS e-Manual DYNAMIXEL Workbench](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)

__Essential 2.__ Clone Github repository
> For Manipuator
```shell
git clone https://github.com/S-CHOI-S/D.A.S.O.M.git
```
> For Palletrone
```shell
git clone https://github.com/S-CHOI-S/D.A.S.O.M-Palletrone.git
```

<br/><br/>

## Usage
__Step 1.__ Control Center GUI
```shell
rosrun dasom_control_gui dasom_app
```
__Step 2.__ Geomagic Touch Device
```shell
roslaunch omni_common omni_state_boundary.launch
```
__Step 3.__ Enable Dynamixel Torque
```shell
roslaunch dynamixel_workbench_controllers torque_ctrl_6DOF.launch
```
__Step 4.__ D.A.S.O.M Manipulator Control
```shell
roslaunch dasom_controllers dasom_manipulator_control.launch
```
__Step 5.__ OptiTrack
```shell
roslaunch optitrack optitrack.launch
```
__Step 6.__ Tf2
```shell
roslaunch dasom_tf2 dasom_tf2_setting.launch
```
__Step 7.__ Camera
```shell
roslaunch dasom_controllers dasom_camera_control.launch
```
__Step 8.__ Palletrone
```shell
roslaunch FAC_MAV FAC_MAV_with_ARM.launch
```
__Step 9.__ Enjoy your flight!!  

<img align="left" src="https://github.com/S-CHOI-S/D.A.S.O.M/assets/113012648/45e7b681-3c64-47b0-baa9-08871c21e14d" width="70%" height="70%"/>  
