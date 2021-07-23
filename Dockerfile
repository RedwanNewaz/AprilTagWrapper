FROM armhf/python:3.6-slim


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