# Research Track assignment 2
In this assignment, we have to develop three nodes:
a. The node implements an action client for setting or cancelling a target (x, y). It utilizes feedback/status from the action server to determine target reaching. Additionally, it publishes robot position and velocity (x, y, vel_x, vel_z) as a custom message, relying on values from the /Odom topic.
b. A service node that, when called, returns the coordinates of the last target sent by the user.
c. Another service node that subscribes to the robot’s position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot’s average speed. 

## Initialisation of the Server-Client
Clone the repository and run the command, given below :
```
./run.sh
```

## Description

### Environment
After that, the environment asks for the desirable value in the x and y coordinates. 
By confirming the desirable position, the robot will move.

You can cancel the program by pressing "C" or "c" . 

### Pseudo Code
```
  Define the Class named ActionClientNode:
  
    Intialise the node class
      -ROS node intialisiation
      -Create action client
      -Subscribe the Odometry
      -Create the necessary publishers

    Define the callback function for the Subscriber "Odometry"
      -Extract the necessary data (Position and Velocity) from the Odometry message
      - Publish the extracted data to the Message

    Define the function for the goal
      -Ask the user for the desire x and y co-ordinates
      -Send the data to action server and publish it in the message. 
      -Ask the user to cancel the program, if he, she or they want. 
      -Wait for the cancellation 
      -Print the final status and the result

In the main block:

if __name__ is "__main__":

  try:
  
    Initialize the ActionClientNode class
    
    Keep the ROS node running
    
  except:
  
    Pass
```
