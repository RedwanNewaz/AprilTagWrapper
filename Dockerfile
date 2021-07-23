FROM python:3.8-slim

WORKDIR /app


RUN apt update

RUN apt install -y git cmake build-essential \
libboost-all-dev libeigen3-dev libgl1-mesa-dev

COPY . .


WORKDIR /app/PoseEKF

RUN ./install.sh

WORKDIR /app



COPY dockerFlaskEKF.py .

CMD ["python3", "dockerFlaskEKF.py", "--K", "camera.param"]