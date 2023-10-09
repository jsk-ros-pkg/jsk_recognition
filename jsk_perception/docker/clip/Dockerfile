FROM pytorch/pytorch:1.7.1-cuda11.0-cudnn8-devel
ARG DEBIAN_FRONTEND=noninteractive
RUN apt -o Acquire::AllowInsecureRepositories=true update \
    && apt-get install -y \
    curl \
    git \
    libopencv-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
# use requirements.txt generetaed by `pip freeze | grep ==` in the container created on 2023-03-31
COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt
RUN pip install git+https://github.com/openai/CLIP.git
RUN mkdir -p /workspace/Clip
COPY server.py /workspace/Clip
ENTRYPOINT cd /workspace/Clip && python server.py