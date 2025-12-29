# Installation Instructions
Following instructions will install the environment for simulation. If you have any questions, open an issue under `aerial_robotics` repository.

## Prerequisites
These instructions assume you have Ubuntu 24.04. installed. Start by installing `git`.
```bash
sudo apt install git
```

## Docker install
To install the Docker engine follow these [instructions](https://docs.docker.com/engine/install/ubuntu/) for installation on Ubuntu. 

Besides official instructions, it is now possible to install docker using the following commands: 
```bash
curl https://get.docker.com | sh \
  && sudo systemctl --now enable docker
```

**GPU support**

In order to be able to use GPU effectively you need to install `nvidia-container-toolkit`. Follow these [instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to install support for GPU.


More details about docker installation can be found [here](https://github.com/larics/docker_files/wiki/2.-Installation)


## Install aerial robotics repository

We recommend creating folder `git` in your home folder for storing various git repositories. In that case, position yourself to that folder `cd ~/git`

1. Clone the repository with
```bash
git clone https://github.com/larics/aerial_robotics.git
```

2. Build the docker
This will create `crazysim_img` image.
```bash
cd ~/aerial_robotics/docker
./build_docker.sh
```

3. Docker first run
**IMPORTANT** Run this script only once. It creates the docker container `crazysim_cont`.
```bash
cd ~/aerial_robotics/docker
./docker_first_run.sh
```
To exit docker container you can press `Ctrl + D`.

4. Starting docker
Any subsequent docker starting should be done with this script as it only starts the `crazysim_cont` container, rather than creating it.
```bash
cd ~/aerial_robotics/docker
./start_docker.sh
```