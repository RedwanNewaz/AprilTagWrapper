FROM python:3.8-slim

WORKDIR /app

RUN apt update

RUN apt install -y git cmake build-essential \
libboost-all-dev libeigen3-dev libgl1-mesa-dev

RUN pip3 install git+https://github.com/RedwanNewaz/AprilTagWrapper.git

COPY PoseEKF ./PoseEKF


WORKDIR /app/PoseEKF

RUN  ./install.sh

WORKDIR /app

COPY example/camera.param .

COPY example/dockerFlaskEKF.py .

CMD ["python3", "dockerFlaskEKF.py", "--K", "camera.param"]