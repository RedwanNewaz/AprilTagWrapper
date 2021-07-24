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
