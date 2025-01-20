FROM pytorch/pytorch:1.13.0-cuda11.6-cudnn8-runtime
ARG DEBIAN_FRONTEND=noninteractive

RUN apt update 
RUN apt install -y git-all \
    && apt install -y build-essential 

RUN pip install ninja \
    && python -m pip install 'git+https://github.com/facebookresearch/detectron2.git'

