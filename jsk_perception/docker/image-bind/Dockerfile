FROM pytorch/pytorch:1.9.1-cuda11.1-cudnn8-devel
ARG DEBIAN_FRONTEND=noninteractive
RUN apt -o Acquire::AllowInsecureRepositories=true update \
    && apt-get install -y \
    curl \
    git \
    libopencv-dev \
    wget \
    emacs \
    python3.8 \
    python3-dev \
    libproj-dev \
    proj-data \
    proj-bin \
    libgeos-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
ENV CUDA_HOME /usr/local/cuda
ENV TORCH_CUDA_ARCH_LIST 8.0+PTX
RUN git clone https://github.com/Kanazawanaoaki/ImageBind.git -b update-data-load
RUN echo 'export CUDA_HOME=/usr/local/cuda' >> ~/.bashrc
RUN echo 'TORCH_CUDA_ARCH_LIST=8.0+PTX' >> ~/.bashrc
RUN pip install flask opencv-python \
    && pip install soundfile \
    && pip install --upgrade pip setuptools wheel \
    && pip install cartopy==0.19.0.post1
RUN cd ImageBind \
    && git pull origin update-data-load \
    && pip install -r requirements.txt \
    && pip install -e .
COPY server.py /workspace/ImageBind
ENTRYPOINT cd /workspace/ImageBind && python server.py