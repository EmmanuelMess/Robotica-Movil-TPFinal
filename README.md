## Levantar

### Instalar Docker
  - [Guia oficial de instalacion de Docker](https://docs.docker.com/engine/install/)

### Construir

```bash
cd covins/docker

make build
```

### Ejecutar servidor y agentes

#### Ejecutar servidor

En una nueva terminal:
```bash
cd covins/docker
./replicate.sh -c
```

En una nueva terminal:
```bash
cd covins/docker
./replicate.sh -s ../covins_comm/config/config_comm.yaml ../covins_backend/config/config_backend.yaml
```

En una nueva terminal:
```bash
cd covins/docker
./replicate.sh -t ../../../codigo ../../../processed/Agente0
export ROS_IP=127.0.0.102
export ROS_MASTER_URI=...
roslaunch src/covins/covins_backend/launch/tf.launch
```

#### Ejecutar un agente

Reemplazar <AGENT_NUMBER> por el numero de agente

```bash
cd covins/docker
export ROS_MASTER_URI=...
./replicate.sh -o ../covins_comm/config/config_comm.yaml ../orb_slam3/Examples/ROS/ORB_SLAM3/launch/launch_docker_ros_euroc.launch <AGENT_NUMBER>
```

```bash
export ROS_MASTER_URI=...
./replicate.sh -p ../../../codigo ../../../processed/Agente<AGENT_NUMBER> <AGENT_NUMBER>
```

#### Ejecutar rviz

Exportar el ROS_MASTER_URI (proveido en la terminal donde se levanto el roscore) y levantar rviz

```
export ROS_IP=127.0.0.102
export ROS_MASTER_URI=...
rviz -d covins/covins_backend/config/covins.rviz
```

#### Abrir una termina en el docker

```bash
./replicate.sh -t ../../../codigo ../../../processed/Agente<AGENT_NUMBER>
```

### Replicar desde los datos crudos

#### Procesado de datos

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
