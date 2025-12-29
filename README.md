# Assignment2

## Guidelines

Develop a ROS2 package that lets a mobile robot endowed with a camera:

- [ ] find all markers in the environment;
- [ ] implement the same approach of assignment 1 -> the robot goes to the one with the lowest ID, it “takes a picture” modifying the image, it moves to the next one.

You have some hints about the position of marker. There are $4$ waypoints:

- $x_1 = -6.0,\, y_1= -6.0$;
- $x_2 = -6.0,\, y_2 = 6.0$;
- $x_3= 6.0,\, y_3 = -6.0$;
- $x_4 = 6.0,\, y_4 = 6.0$.

That the robot can visit to detect each marker.

Implement the assignment both in simulation (the world file assignment2.world is given). Possibly you can also try to implement the approach with the real robot.

### Requirement:
- [ ] you should use PlanSys2 to plan the actions of the robot
- [ ] Create one (or more, if needed) ROS2 packages that implement the requested behavior.
- [ ] Publish the new package on your own repository (one for group is ok)
- [ ] Create a comprehensive ReadMe,
- [ ] Add a video to your ReadMe, showing the behaviour of your code (if the approach is implemented also with real robot, please add a video also showing that)
