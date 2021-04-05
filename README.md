# pongBot
Progress on building a robot that can bounce a ping-pong ball forever

## TODO
- [x] Find cool project
- [x] Design crude sketch, specifying DOF, # of links, actuators, and end effector
  - 4 DOF, 2 links, 4 actuators, and a paddle as the end effector
- [x] Figure out and purchase the actuators necessary to achieve the desired effect
  - Dynamixel AX-12a
- [ ] 3D model necessary links, base, and other parts to have a 3D printable prototype
- [ ] Assemble and wire parts together
- [ ] Write driver code to be able to control the robot in a controller/agent configuration
- [ ] Figure out coordinate frame transformations of each necessary part (base, joints, end effector)
- [ ] Use OpenCV to identify and get pose estimation of a ping pong ball
- [ ] Figure out coordinate frame transforamtions of camera to robot
- [ ] Be able to have robot autonomously move its end effector to a static ping-pong ball identified by the overhead camera
- [ ] Use physics equations and magic to calculate the force and angle necessary to send the ping-pong ball vertically from a static position of resting on the paddle/end effector
- [ ] Use said magic to move the end effector to the location of the ball coming down from the initial hit and strike the ball in a controlled manner such that the ball will land somewhere within the robot's range of motion
- [ ] Repeat previous step 
