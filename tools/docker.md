## **Docker**
- ### **Description**: the system we use to run our code in through virtualized containers
- ### **General Usage**:
    - `docker ps -a`: list containers
    - `docker start [id]`: start the container with the id/name `[id]`
    - `docker exec -it [id] /bin/bash`: open a shell inside the container with the id/name `[id]`
    - `exit`: exit the container
- ### **Updating Container**:
    - `exit` your docker container in all windows
    - `docker stop [id]`: stop your docker container
    - `docker rm [id]`: remove your docker container
    - `cd ~/2023RobotCode/docker`: change directories to Docker folder of our robot code
    - `docker image rm frc900/zebros-2023-dev`: container images are big, delete the old one if you're running low on space
    - `docker pull frc900/zebros-2023-dev:latest`: download the new container (this will take a while)
    - `./docker-run`: create and enter a new container
- ### **Notes**:
    - The username/password inside Docker is `ubuntu`/`ubuntu`

[Home](/README.md)