# Hockey Dribbling bot 

This is the readme file containing instructions on how to run the hockey dribbling bot.

The hockey bot implementation has been built on top of the existing triton platform (repo).


## Steps to run 


1. Start container (issue command): This script spins up the docker container and launches the triton launch file
```
  ./triton_start.sh
```
2. Open a new terminal window

3. Log into the container
```
sudo docker compose exec triton_noetic bash
``` 
4. Run the scripts
```
# navigate to the src folder 
cd /catkin_ws/src

# run the main file 
python3 robotDribble.py
```

## Additional libraries added (in the Dockerfile)
* ros-noetic-cv-bridge 

## Things to note
* In certain occassions, it was observed that the camera tends to not get detected. In such situations, reboot the triton bot and re-attempt. 


