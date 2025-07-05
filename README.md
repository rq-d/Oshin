
# Autonomy Code

Docker-compose file runs two containers; one with ROS and another with Ubuntu. Source code will be placed in `app/` and mounted to the ubuntuROS container.

To run:
````
docker-compose up
docker exec -it oshin-UbuntuROS-1 bash
````

Use your browser to access the running web interface at `localhost:8080`