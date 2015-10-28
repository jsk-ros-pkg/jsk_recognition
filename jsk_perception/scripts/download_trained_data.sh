#!/bin/sh

mkdir -p $(rospack find jsk_perception)/trained_data
TRAINED_DATA_DIR=$(rospack find jsk_perception)/trained_data

# download svm trained_data for drc drill task
wget --timestamping "https://drive.google.com/uc?id=0B5hRAGKTOm_KWW11R0FTX0xjTDg&export=download" -O $TRAINED_DATA_DIR/drill_svm.xml

# downalod bof & logistic regression trained_data for apc recognition sample
gdown "https://drive.google.com/uc?id=0B9P1L--7Wd2vemVRaDBOWDVpb28&export=download" -O $TRAINED_DATA_DIR/apc2015_sample_bof.pkl.gz
gdown "https://drive.google.com/uc?id=0B9P1L--7Wd2veFY5ZFNqbzAzNmc&export=download" -O $TRAINED_DATA_DIR/apc2015_sample_clf.pkl.gz
