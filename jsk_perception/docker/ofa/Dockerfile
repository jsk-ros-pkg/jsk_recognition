FROM pytorch/pytorch:1.8.1-cuda11.1-cudnn8-devel
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
RUN sed -i 's/pycocotools==2\.0\.4/pycocotools==2.0.6/g' /tmp/requirements.txt && pip install -r /tmp/requirements.txt
RUN git clone https://github.com/OFA-Sys/OFA \
    && mkdir -p /var/mount
RUN cd OFA \
    && git checkout 1809b55 \
    && sed -i 's/pycocotools==2\.0\.4/pycocotools==2.0.6/g' requirements.txt \
    && pip install -r requirements.txt \
    && pip install "protobuf<3.20.0" "numpy>=1.20.3,<1.24" "pytorch-lightning==1.7.7" "torch==1.8.1" flask
COPY server.py /workspace/OFA
ENTRYPOINT cd /workspace/OFA && python server.py
