this is a simple ros2 workspace.
src/my_pubsub has main code:

build:
  colcon build
node: 
talker, send cmd to listner:
  ros2 run my_pubsub undt (control with numbers input)
  or
  ros2 run my_pubsub boatt (manual control with arrows)

listener: (pi side)
  login to raspberrypi:
    ssh -X -o ServerAliveInterval=60 -p 80 jackal@134.209.218.187
    username: piboat
      password: pi
  
  docker exec -it cs1 bash
    source install/setup.bash
    ros2 run my_pubsub undl
    or
    ros2 run my_pubsub boatl
      you should hear a click (relay)

PWM signals sent to GPIOs 12 and 13 to control boat
GPIO 18 is where we listen to the turn duty cycle to determine if we are using remote or pi (0 or 7.5 we use pi and any other we use remote)(7.5 or 0 is when remote is off or when st knob is in middle and sr is all the way to right when not pressing anything)
GPIOs 20 and 21 are used for the relays to toggle between transmitter use and raspberry pi use

