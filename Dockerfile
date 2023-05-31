FROM ubuntu:18.04

ARG DEBIAN_FRONTEND=noninteractive

ENV PROJECT_DIR="/UcoSLAM/"

RUN apt-get update \
    && apt-get install -y \
	build-essential \
    cmake \
    libopencv-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libopenni2-dev

COPY . $PROJECT_DIR

RUN cd $PROJECT_DIR \
    && mkdir build \
    && cd build \
    && cmake ../ -DBUILD_GUI=ON \
    && make -j4