## File Locations:
The custom launch files are located in the directory:
- /home/scoutmini2/scout_ws/src/custom_launch/launch

## Launch:
To launch one of these files, you can one of the following commands:
- ros2 launch custom_launch <custom_launch_file_name.py>
- Slam

The first is a ros command to launch the file, while the second is a bash command shortcut for the launches.

The second option has different forms:
- Slam (launches Livox Slam)
- SlamALL (launches Livox and Camera Slam)
- Nav2 (launches livox nav2)
- Nav2ALL (launches livox and camera Nav2)
- Nav2Map (launches livox Nav2 loading a map)
- Nav2MapALL (launches livox and camera Nav2 loading a map)

## Editing a launch file:
You can edit the launch files, for example, change the map file path in a Map launch to load the desired map. After changes, you must:
- cd scout_ws
- colcon build
- source install/setup.bash
