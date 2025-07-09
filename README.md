
# Autonomy Code

Docker-compose file runs two containers; one with ROS and another with Ubuntu. Source code will be placed in `app/` and mounted to the ubuntuROS container.

## Quickstart
### CLI
if first time; make the new docker image
````
cd docker
docker build -t ardunode:latest .
````

To run:
````
docker-compose up
docker exec -it oshin-oshin1-1 bash
mavproxy.py --aircraft test --master=:14550
````

Now we have console from which we can send mavlink commands to control the vehicle ie;
````
mode guided
arm throttle
takeoff 200
````

Watch the position change over time in another terminal
````
ros2 topic echo /ap/pose/filtered
````

### Visualization
The dock-compose file spins up a container with gzweb, which is a web interface to gazebo. Use your browser to access the running web interface at `localhost:8080`

# Notes
### Info on the images
https://hub.docker.com/r/arm64v8/ros/

### Misc Sources
Similar project found on youtube
https://www.youtube.com/watch?v=OA5Id9u_H58
https://github.com/Intelligent-Quads/iq_sim#
https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html


## Ros Setup Notes
must be done every new shell or added to `~/.bashrc`
````
export ROS_DOMAIN_ID=99 
````

## TODO
- [ ] mount an example python script that does the following
  - [ ] arm copter
  - [ ] takeoff to 200m
  - [ ] move in some direction
- [ ] set up gazebo interface for visualization
  - [ ] try running interface in the ardunode container and publish over network to gzweb
- [ ] switch to a ground or USV platform
  - [ ] might require a local gazebo installation or some config within gzweb interface