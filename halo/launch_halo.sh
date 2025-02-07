#!/bin/bash

docker-compose build --build-arg USER_ID=$(id -u) --build-arg USER_NAME=$(whoami) --build-arg GROUP_ID=$(id -g) -t halo-image .
docker compose up