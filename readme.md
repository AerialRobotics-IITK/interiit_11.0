# PlutoX challenge

This is the repository for the Drona aviation PS in the InterIIT Techmeet 11.0

The two files currently in the repository show how to simply arm the drone, with an initial roll, pitch, yaw and thrut of 1500, 1500, 1500, 900 respectively.

The wrapper we will build will be as a python package. Please look that up. Currently there is of course, no package. But it needs to integrted at the earliest.

There are a few main functionalities that the wrapper must contain. Issues will be opened up for them. Please claim your issues. The key functionalitites are :

Set roll, pitch, yaw or thrust. This functionality is already implemented, but as of now it takes inputs in the 900-2100 range. It needs to be mapped to the conventional units, for example, radians for the angles. 

Takeoff

Land

There is currently no use of queries (OUT packets) as well. This support should be added as it will be important for the main problem statement and the functionalities outlined above.

To run the code, just run ```example.py``` after connecting to Pluto's hotspot.