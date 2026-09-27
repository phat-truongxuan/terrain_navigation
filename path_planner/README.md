to launch map:
    ros2 launch path_planner map.launch.py 

to run path planning:
    ros2 run path_planner nav_main

to publish all in one message:
    
    ros2 topic pub --once /all_in_one path_planner/msg/AllInfo "{start_id_x: 2, start_id_y: 20, goal_id_x: 35, goal_id_y: 25, omega: 1, beta: 0, gamma: 100, mu: 0.3, color: 7, follow_path: True}"


custom map create in Blender: 
https://www.youtube.com/watch?v=GNbH8Pf7nGk

map sizing note:
example: terrain4.xyz has x y z ranging from -1 to 1, normalize to 0 and 2, therefor this map will be 2x2 meter size, size adjust will magnify this. New mpa will have the size of 2xsize_adjust