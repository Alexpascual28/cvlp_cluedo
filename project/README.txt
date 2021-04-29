COMP3631 Project - Group 3

This README provides instructions to run the developed code for the COMP3631 Group Project.


Input Files

First modify the input_points.yaml in the example/ directory. Change the points accordingly.


Python Scripts

There are four main scripts in the src/ directory:
- el17m3m/identifier.py
- el17m3m/room_finder.py
- el17m3m/turtlebot_control.py
- el18apsr/find_cluedo_rand.py

The scripts can be run in any order except for the turtlebot_control.py, which should be run last.

Then use the following commands within the project/ directory:

python src/el18apsr/find_cluedo_rand.py

python src/el17m3m/identifier.py

python src/el17m3m/room_finder.py

python src/el17m3m/turtlebot_control.py
