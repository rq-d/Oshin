
# Autonomy Code

Docker-compose file runs two containers; one with ROS and another with Ubuntu. Source code will be placed in `app/` and mounted to the ubuntuROS container.

To run:
````
docker-compose up
docker exec -it oshin-UbuntuROS-1 bash
````

if first time; make the new docker image
````
cd docker
docker build -t test:latest .
````

Use your browser to access the running web interface at `localhost:8080`

## Info on the images
https://hub.docker.com/r/arm64v8/ros/

## Sources
Similar project found on youtube
https://www.youtube.com/watch?v=OA5Id9u_H58
https://github.com/Intelligent-Quads/iq_sim#
https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html


## Ros Setup Notes
must be done every new shell or added to `~/.bashrc`
````
source /opt/ros/rolling/setup.bash
export ROS_DOMAIN_ID=99 
````