## Levantar

### Instalar Docker
  - [Guia oficial de instalacion de Docker](https://docs.docker.com/engine/install/)

### Construir

```bash
cd ccm_slam/docker

make build
```

Copiar los datos de rosbags.7z a la carpeta padre del repo.

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
docker compose -p agent_${AGENT_NUM} -f docker-compose.client.yml up
```

Esperar a que el cliente levante.

En una nueva terminal:
```bash
cd covins/docker
export AGENT_NUM=<AGENT_NUMBER>
export ROS_MASTER_URI=http://localhost:11311/
docker compose -p agent_${AGENT_NUM} -f docker-compose.data.yml up
```

#### Ejecutar rviz

Exportar el ROS_MASTER_URI (proveido en la terminal donde se levanto el roscore) y levantar rviz

```
export ROS_MASTER_URI=http://localhost:11311/
rviz -d ccm_slam/cslam/conf/rviz/ccmslam.rviz
```
