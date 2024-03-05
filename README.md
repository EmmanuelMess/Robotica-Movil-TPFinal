## Levantar

### Instalar Docker
  - [Guia oficial de instalacion de Docker](https://docs.docker.com/engine/install/)

### Construir

```bash
cd ccm_slam/docker

make build
```


### Ejecutar servidor y agentes

#### Ejecutar servidor

En una nueva terminal:
```bash
cd ccm_slam/docker
export ROS_MASTER_URI=http://localhost:11311/
docker compose -f docker-compose.server.yml up
```

Esperar a que el servidor levante.

#### Ejecutar un agente

Reemplazar <AGENT_NUMBER> por el numero de agente

En una nueva terminal:
```bash
cd ccm_slam/docker
export ROS_MASTER_URI=http://localhost:11311/
export AGENT_NUM=<AGENT_NUMBER>
docker compose -f docker-compose.client.yml up
```

Esperar a que el cliente levante.

En una nueva terminal:
```bash
cd covins/docker
export AGENT_NUM=<AGENT_NUMBER>
export ROS_MASTER_URI=http://localhost:11311/
docker compose -f docker-compose.data.yml up
```

#### Ejecutar rviz

Exportar el ROS_MASTER_URI (proveido en la terminal donde se levanto el roscore) y levantar rviz

```
export ROS_IP=127.0.0.102
export ROS_MASTER_URI=http://localhost:11311/
rviz -d covins/covins_backend/config/covins.rviz
```

#### Abrir una termina en el docker

```bash
./replicate.sh -t ../../../codigo ../../../processed/Agente<AGENT_NUMBER>
export ROS_MASTER_URI=http://localhost:11311/
```

### Replicar desde los datos crudos

#### Procesado de datos

```
docker run -it --rm --net=host \
        --volume "/home/emmanuelm/Rápido/Projects/GitHub/TPFinal/raw-data:/root/covins_ws/dataset" \
        --workdir "/root/covins_ws/dataset" \
        osrf/ros:melodic-desktop-bionic \
        /bin/bash
```

```bash
docker run -it --rm --net=host \
        --volume "/home/emmanuelm/Rápido/Projects/GitHub/TPFinal/raw-data:/root/covins_ws/dataset" \
        --workdir "/root/covins_ws/dataset" \
        osrf/ros:humble-simulation-jammy \
        /bin/bash

sudo apt update
sudo apt install python3-pip
python3 -m pip install rosbags==0.9.19
```
