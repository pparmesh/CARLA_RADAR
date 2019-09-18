Run RADAR
The script titled ClientRADAR request information of the vehicle object and
produces the required transformations in the world frame and the ego vehicle frame.
This is just a helper script that used by RADAR.py

This script can also be used to get 3D bounding box of the vehicle in the world frame (getGlobalBB)
as well as in the ego vehicle frame getBBEGo(ego_vehicle, vehicle). Here the ego vehicle frame is 
defined as the geometrical center of the vehicle of the vehicle. Appropriate transformations are needed in order to 
get it in the frame of the relevant sensor. 

The script titled RADAR emulates a RADAR in Carla. It has a RADAR class that instantiates a RADAR
object and a vehicle class. It first makes list of all vehicles in the simulator environment.
It then eliminates those not in my RADAR FOV (both angular and rectilinear).
It also eliminates occluded vehicles. It adds a certain random noise to the pose values.
The visualization function is to display a scatter plot of vehicles in the environment, detected
vehicles and ego vehicle. It also shows my RADAR FOV.
NOTE: Visualization not scaled for equation of line.
