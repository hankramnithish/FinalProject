The "final_project" is the package that contains the launch files and all the python code.

 final_project --> final_project --> robot_recognition    - TO RUN THE YOLO MODEL NODE.
 
 final_project --> final_project --> robot_control        - TO RUN THE CONTROLLER NODE.
 
 final_project --> final_project --> obstacle_avoidance   - TO RUN THE NODE TO PROCESS LIDAR.
 
 final_project --> final_project --> image_sub_save       - TO RUN THE NODE THAT SAVES IMAGES FROM ROS TOPICS
 
 final_project --> launch -->  run_nodes.launch.py        - TO RUN THE ABOVE THREE NODES.
 
    
 




The "sensor.launch.py" is used to launch all the camera and lidar_sensor while inside the Turltebot4 terminal. 
