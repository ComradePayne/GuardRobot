# GuardRobot
AMR '17 Final Project

This project concerns the programming of a "watch-dog" routine for a TurtleBot. It uses the Kinect to recognize certain features of a given object (for our implementation, color) and categorizes this object, behaving differently towards different categories. 

There are three categories thus far: Guard, Enemy, and Waypoint.

Guard: The thing to defend, identified by the color Blue. If the robot sees an Enemy near this, the robot will follow the Enemy and attempt to drive it off with a cacophony of beeps or barking.
Enemy: The thing to drive off, identified by the color Red. Anyone wearing bright red shoes had better beware!
Waypoint: The things to attempt to patrol between, identified by the color Green. The robot will use these as patrol markers, moving between them in sequence.

