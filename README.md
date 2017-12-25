# RemoteDualUR5
Proof of Concept exchaning the pose of a robot between two masters, to be able to have two robots work together without them beiing controlled by the same controller.


THe udp_pose.py script runs as a ROS node and sends the pose of robot2 (in this case a UR5) over udp to the other script.
The collisionadder.py script receives this information and uses it to place a collision model of robot2 in the planning scene of robot1.
This allows robot1 to plan with collision avoidance as to not crash into robot2.
