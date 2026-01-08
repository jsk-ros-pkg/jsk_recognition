FROM pytorch/pytorch:1.13.1-cuda11.6-cudnn8-devel
ARG DEBIAN_FRONTEND=noninteractive
RUN apt -o Acquire::AllowInsecureRepositories=true update \
    && apt-get install -y \
    curl \
    git \
    libopencv-dev \
    wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
ENV CUDA_HOME=/usr/local/cuda
ENV TORCH_CUDA_ARCH_LIST=8.0+PTX
RUN git clone https://github.com/IDEA-Research/GroundingDINO.git
RUN echo 'export CUDA_HOME=/usr/local/cuda' >> ~/.bashrc
RUN echo 'export TORCH_CUDA_ARCH_LIST=8.0+PTX' >> ~/.bashrc
RUN cd GroundingDINO \
    && pip install -r requirements.txt \
    && pip install -e .
RUN pip install flask opencv-python \
    && pip install "numpy>=1.20,<2"
RUN mkdir -p GroundingDINO/weights \
    && cd GroundingDINO/weights \
    && wget -q https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth \
    && wget -q https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha2/groundingdino_swinb_cogcoor.pth
COPY server.py /workspace/GroundingDINO
ENTRYPOINT cd /workspace/GroundingDINO && python server.py