#!/bin/bash 

sudo docker compose up -d 

sudo docker compose exec triton_noetic bash -c "source devel/setup.bash && roslaunch stingray_camera triton.launch"
