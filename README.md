# Docker for Running ROS Noetic

### Clone Repository
```bash
cd ~
git clone https://gitlab.com/HCRLab/stingray-robotics/stingray_docker.git
```

### Move catkin_ws
In order to mount our `catkin_ws` directory within the docker container, we need to move it inside the newly cloned `stingray_docker` directory (due to docker permissions). Note that if we move `catkin_ws` as shown in the next line, we may need to run `catkin_make clean` (or `catkin clean`) before running `catkin_make` (or `catkin build`) again due to changes in the path. 
```bash
mv ~/catkin_ws ~/stingray_docker/catkin_ws
catkin clean
catkin build
```


### Download the Docker Image
The pre-built docker image with ROS Noetic (and Ubuntu 20.04) can be downloaded here: https://drive.google.com/file/d/1sI_Api04NNbVwY2xCfx6aEqlrVwID7R-/view?usp=sharing. Save the `.tar.gz` inside the `stingray_docker` directory. 

### Load and Run the Docker Image
Inside the `stingray_docker` directory, load the docker image only during the first run. Then you can run `docker-compose` to run the docker image directly for future runs. 
```bash
run docker load -input triton_noetic.tar.gz
docker-compose up
```

Based on the `docker-compose.yaml` file, when we run this docker image, we are mounting the `catkin_ws/src` directory inside the docker container. This allows us to directly edit the files from outside the docker container, and just run ROS launch files or Python files from inside the docker container. 

Once the docker image is running, you can check its status by running `docker ps -a`. This image in particular does not run anything inside; you must run commands from inside the container. 

To do so, you can open a new "terminal": 
```bash
docker-compose exec triton_noetic bash
```

Inside this virtual terminal, you can run `catkin build` or `source devel/setup.bash` directly. Afterwards, you can run `roslaunch` or `rosrun`. 

### Important Things to Note
The docker container is generally headless, meaning we ordinarily do not have GUI access (without complicated workarounds). This means we cannot run RViz or Gazebo inside the container easily. Since the `docker-compose.yaml` mirrored the networking from outside and inside docker, we can use the ROS Melodic instance outside docker to run some other packages (including RViz) that do not have much changes between Melodic and Noetic. 

You can check the `Dockerfile` to see what dependencies have been installed in the pre-built image provided. If you need other dependencies, you can directly install it inside the docker container (or even create your own image instead). You may also opt to persist the images so that you do not need to reinstall any newly installed dependencies. 

One of the main reasons to work inside the docker container is to natively use Python3 (including all Python3 libraries such as modern PyTorch), because ROS Melodic is natively built for Python2. However, inside the docker container, we do not have CUDA acceleration (this is only possible for docker containers that were built with the NVIDIA Jetpack which is still Melodic; although we can install Noetic in such a container, additional packages will have to be built from source instead of just an `apt install`). If you really need CUDA acceleration, you may opt to run that specific application outside the docker container. Generally working with NVIDIA Jetson and CUDA acceleration takes nontrivial effort. 

