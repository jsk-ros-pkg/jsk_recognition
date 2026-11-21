# FROM pytorch/pytorch:1.7.1-cuda11.0-cudnn8-devel
FROM pytorch/pytorch:1.9.1-cuda11.1-cudnn8-devel
# FROm pytorch/pytorch:1.13.1-cuda11.6-cudnn8-devel
ARG DEBIAN_FRONTEND=noninteractive
RUN apt -o Acquire::AllowInsecureRepositories=true update \
    && apt-get install -y \
    curl \
    git \
    libopencv-dev \
    wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
ENV CUDA_HOME /usr/local/cuda
ENV TORCH_CUDA_ARCH_LIST 8.0+PTX
RUN git clone https://github.com/IDEA-Research/GroundingDINO.git
RUN echo 'export CUDA_HOME=/usr/local/cuda' >> ~/.bashrc
RUN echo 'TORCH_CUDA_ARCH_LIST=8.0+PTX' >> ~/.bashrc
RUN pip install flask opencv-python \
    && pip install "numpy>=1.20"
RUN cd GroundingDINO \
    && pip install -r requirements.txt \
    && pip install -e .
RUN mkdir -p GroundingDINO/weights \
    && cd GroundingDINO/weights \
    && wget -q https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
COPY server.py /workspace/GroundingDINO
ENTRYPOINT cd /workspace/GroundingDINO && python server.py