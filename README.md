## Setup

### Install Docker
  - [Docker official Installation guide](https://docs.docker.com/engine/install/)

### Run the setup script

```bash
cd covins/docker

./replicate.sh -c
./replicate.sh -t ../../../raw-data/Agente1

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
