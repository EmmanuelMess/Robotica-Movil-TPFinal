## Setup

### Install Docker
  - [Docker official Installation guide](https://docs.docker.com/engine/install/)

### Run the setup script

```bash
cd covins/docker

./replicate.sh -c
./replicate.sh -s ../covins_comm/config/config_comm.yaml ../covins_backend/config/config_backend.yaml
./replicate.sh -o ../covins_comm/config/config_comm.yaml ../orb_slam3/Examples/ROS/ORB_SLAM3/launch/launch_docker_ros_euroc.launch 0
./replicate.sh -t ../../../codigo ../../../euroc-fake
./replicate.sh -v # Only Xorg
```

### Procesado de datos

./replicate.sh -o ../covins_comm/config/config_comm.yaml ../orb_slam3/covins_examples/euroc_examples_mh1.sh ../../../euroc

```
docker run -it --rm --net=host \
        --volume "/home/emmanuelm/Rápido/Projects/GitHub/TPFinal/raw-data:/root/covins_ws/dataset" \
        --workdir "/root/covins_ws/dataset" \
        osrf/ros:melodic-desktop-bionic \
        /bin/bash
```

```bash
sudo apt update
sudo apt install python3-pip
python3 -m pip install rosbags

docker run -it --rm --net=host \
        --volume "/home/emmanuelm/Rápido/Projects/GitHub/TPFinal/raw-data:/root/covins_ws/dataset" \
        --workdir "/root/covins_ws/dataset" \
        osrf/ros:humble-simulation-jammy \
        /bin/bash
```
