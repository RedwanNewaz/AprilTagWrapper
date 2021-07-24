FROM alpine AS builder

# Download QEMU, see https://github.com/docker/hub-feedback/issues/1261
ENV QEMU_URL https://github.com/balena-io/qemu/releases/download/v3.0.0%2Bresin/qemu-3.0.0+resin-arm.tar.gz
RUN apk add curl && curl -L ${QEMU_URL} | tar zxvf - -C . --strip-components 1
FROM arm32v7/python:3.8


WORKDIR /app


RUN apt update

RUN apt install -y git cmake build-essential \
libboost-all-dev libeigen3-dev libgl1-mesa-dev

COPY . .

RUN python3 setup.py install 

WORKDIR /app/PoseEKF

RUN ./install.sh

WORKDIR /app



COPY dockerFlaskEKF.py .

CMD ["python3", "dockerFlaskEKF.py", "--K", "camera.param"]
