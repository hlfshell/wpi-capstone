default:
    just --list

# ======================
# Python Code
# ======================

# lint python project code
lint:
    flake8 ros_ws/src

isort:
    isort ros_ws/src

black:
    black ros_ws/src

fix: isort black lint

# ======================
# Docker
# ======================

docker-build:
    docker build -t capstone-ros .

docker-rm-image:
    docker image rm -f capstone-ros

docker-halt:
    docker stop capstone-ros

docker-rebuild: docker-rm-image docker-build

# docker ros2 core
docker-ros:
    docker ps | grep capstone-ros >/dev/null || \
    docker run \
        -it --rm \
        --name capstone-ros \
        --mount type=bind,source=./ros_ws,target=/ros_ws \
        --mount type=bind,source=./.code-server,target=/code-server \
        --mount type=bind,source=./envs/.bashrc,target=/root/.bashrc \
        capstone-ros \
        bash

# docker bash shell into core ros2 server (if the core is not running, this will be the core)
docker-bash: docker-ros
    docker exec -it capstone-ros /bin/bash 

# docker vs code environment
docker-vs-code: docker-ros
    docker exec -it capstone-ros code tunnel \
        --name capstone-ros --accept-server-license-terms \
        --cli-data-dir /code-server --no-sleep

# ======================
# VM
# ======================

vm-start:
    vagrant up

vm-stop:
    vagrant halt

vm-rebuild:
    vagrant destroy -f
    vagrant up

vm-bash: vm-start
    vagrant ssh

# Run 
vm-vs-code: vm-start
    vagrant ssh -c "code tunnel \
            --name capstone-ros-vm \
            --accept-server-license-terms \
            --no-sleep \
            --cli-data-dir /home/vagrant/.code_server"