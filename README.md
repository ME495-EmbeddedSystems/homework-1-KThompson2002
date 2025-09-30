# ME495 Embedded Systems Homework 1
Author: Kyle Thompson
1. Use `ros2 launch turtle_control waypoints.launch.xml` to run the code
2. The `ros2 service call /load turtle_interfaces/srv/Waypoint '{waypoints: [{x: 3.9, y: 5.4, z: 0.0}, {x: 1.4, y: 1.6, z: 0.0}, {x: 2.2, y: 9.4, z: 0.0}, {x: 7.2, y: 6.1, z: 0.0}, {x: 4.0, y: 2.6, z: 0.0}, {x: 8.2, y: 1.5, z: 0.0}]}'` service loads waypoints for the turtle to follow
3. The `ros2 serviccall /toggle std_srvs/srv/Empty` starts and stops the turtle.
4. Here is a video of the turtle in action.

[turtle_sim_record.webm](https://github.com/user-attachments/assets/c35d9681-64de-4e6c-a40c-22aee0febe81)


5. Here is a video when the bag is played.

[turtle_bag_video.webm](https://github.com/user-attachments/assets/5e313aa0-819b-4dbc-901f-dc5ef76c02fb)
