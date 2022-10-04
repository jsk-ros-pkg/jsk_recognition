#
# Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
#

# api directories
OFAPROJECT = ofa
CLIPPROJECT = clip
GLIPROJECT = glip
# image names
OFAIMAGE = jsk-ofa-ros-server
CLIPIMAGE = jsk-clip-ros-server
GLIPIMAGE = jsk-glip-ros-server
# commands
BUILDIMAGE = docker build
REMOVEIMAGE = docker rmi
# configuration directories
CONFIGDIR = $(shell echo ${HOME}/.jsk_vil_apis)
PARAMDIR = $(CONFIGDIR)/params
# Download files
PARAMURLS = parameter_urls.txt
# OFA parameters
OFAPARAMFILES = $(foreach param, $(OFAPARAMS), $(PARAMDIR)/$(param))

# TODO check command wget exists, nvidia-driver version

# TODO add clip, glip
download:
	@if [ ! -d $(PARAMDIR) ]; then\
		mkdir -p $(PARAMDIR); \
	fi
	wget -c -i $(PARAMURLS) -P $(PARAMDIR)

ofa: download
	$(BUILDIMAGE) $(OFAPROJECT) -t $(OFAIMAGE) -f $(OFAPROJECT)/Dockerfile

all: ofa

# TODO add clip, glip
clean:
	@$(REMOVEIMAGE) $(OFAIMAGE)

wipe: clean
	rm -fr $(CONFIGDIR)

.PHONY: clean wipe
