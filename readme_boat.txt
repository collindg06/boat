this is a simple ros2 workspace.
src/my_pubsub has main code:

build:
  colcon build
node: 
talker, send cmd to listner
  ros2 run my_pubsub undt

listener: (pi side)
  login to raspberrypi:
    jackel
    piboat
      passwd pi
  
  docker exec -it cs1 bash
    source install/setup.bash
    ros2 run my_pubsub undl 
      you should hear a click (relay)
