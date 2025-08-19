# uchariot_lidar_workspace
Repository of my code development while working on the NASA robot "Micro Chariot" Lidar Awareness and object detection

lidarSubscriber.py - first Python code I wrote for the lidar, it was my first time getting experience on creating a data stream from the LiDAR to the command prompt
csvcreator.py - takes the data from the lidar and stores it in a csv file(Obsolete, replaced by lidarRunner 1-3)
lidarRunner1-3: Incorporates starting the lidar and managing lifestyle command,s and different methods of data collection, from ordering data based on time on 1 and 2, to adding data one 'spin' at a time and auto-deleting the oldest data on 3
lidar_watcher.py - examines the csv file and displays a message on the command prompt when the distance measured in a specific angle decreases below a specific threshold set in the code
"Setting Up The Lidar Development Workspace For Windows" - A File that I and other developers created to guide interns in creating and using a proper computer environment for LiDAR developemnt
