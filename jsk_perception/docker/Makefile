#
# Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
#

# api directories
OFAPROJECT = ofa
CLIPPROJECT = clip
# image names
OFAIMAGE = jsk-ofa-server
CLIPIMAGE = jsk-clip-server
# commands
BUILDIMAGE = docker build
REMOVEIMAGE = docker rmi
# configuration directories
ifdef ROS_HOME
CONFIGDIR = $(ROS_HOME)/data/jsk_perception
else
CONFIGDIR = $(HOME)/.ros/data/jsk_perception
endif
PARAMDIR = $(CONFIGDIR)/vil_params
# Download files
PARAMURLS = parameter_urls.txt
# OFA parameters
OFAPARAMFILES = $(foreach param, $(OFAPARAMS), $(PARAMDIR)/$(param))

all: ofa clip

# TODO check command wget exists, nvidia-driver version

# TODO add clip
$(PARAMDIR)/.download:
	@if [ ! -d $(PARAMDIR) ]; then\
		mkdir -p $(PARAMDIR); \
	fi
	wget -c -i $(PARAMURLS) -P $(PARAMDIR)
	touch $(PARAMDIR)/.download

ofa: $(PARAMDIR)/.download
	$(BUILDIMAGE) $(OFAPROJECT) -t $(OFAIMAGE) -f $(OFAPROJECT)/Dockerfile

clip: $(PARAMDIR)/.download
	$(BUILDIMAGE) $(CLIPPROJECT) -t $(CLIPIMAGE) -f $(CLIPPROJECT)/Dockerfile

# TODO add clip, glip
clean:
	@$(REMOVEIMAGE) $(OFAIMAGE)

wipe: clean
	rm -fr $(PARAMDIR)

.PHONY: clean wipe ofa clip
