# Woowa - MORAI Simulation Project
---
## 💻 팀원
<table>
    <tr height="150px">
        <td align="center" width="130px">
            <a href="https://github.com/syw2045"><img height="100px" width="100px" src="https://avatars.githubusercontent.com/u/81313733?v=4"></a>
            <br/>
            <a href="https://github.com/syw2045">SSong</a>
        </td>
        <td align="center" width="130px">
            <a href="[https://github.com/SaltnLight-pet"><img height="100px" width="100px" src="https://avatars.githubusercontent.com/u/142612336?v=4"></a>
            <br/>
            <a href="https://github.com/SaltnLight-pet">SaltnLight-pet</a>
        <td align="center" width="130px">
            <a href="https://github.com/Heesun0-0"><img height="100px" width="100px" src="https://avatars.githubusercontent.com/u/125299969?v=4"/></a>
            <br/>
            <a href="https://github.com/Heesun0-0">Heesun0-0</a>
        <td align="center" width="130px">
            <a href="https://github.com/JunseoMin"><img height="100px" width="100px" src="https://avatars.githubusercontent.com/u/114414158?v=4"/></a>
            <br/>
            <a href="https://github.com/JunseoMin">JunseoMin</a>
        </td>
    </tr>
</table>

--- 

## Enviroment

|OS|Version|
|:---:|:---:|
|Ubuntu|20.04.6 LTS|
|ROS|Noetic|
|MORAI|230911.S2.woowa5|

## Mapping
```
roslaunch velodyne_pointcloud VLP16_points.launch
rosrun changmin convert_gps.py
roslaunch hdl_graph_slam hdl_graph_slam.launch
rviz
```

## Localization
```
rosrun changmin convert_gps.py
rosrun changmin make_odom.py

roslaunch velodyne_pointcloud VLP16_points.launch
roslaunch hdl_localization hdl_localization.launch

rviz -d hdl_localization.rviz
```
