#!/bin/sh

mkdir -p $(rospack find jsk_perception)/trained_data
wget --timestamping "https://drive.google.com/uc?id=0B5hRAGKTOm_KWW11R0FTX0xjTDg&export=download" -O $(rospack find jsk_perception)/trained_data/drill_svm.xml
