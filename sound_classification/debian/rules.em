#!/usr/bin/make -f
# -*- makefile -*-

# Copied from https://github.com/ros-infrastructure/bloom/blob/master/bloom/generators/debian/templates/catkin/rules.em
# * Change install prefix /opt/ros -> /opt/gitai/ros
# * Add cmake config for ccache
# * Disable DCATKIN_BUILD_BINARY_PACKAGE="1" for building the catkin package
# * Skip error on dh_shlibdeps / dh_strip

# Sample debian/rules that uses debhelper.
# This file was originally written by Joey Hess and Craig Small.
# As a special exception, when this file is copied by dh-make into a
# dh-make output file, you may use that output file without restriction.
# This special exception was added by Craig Small in version 0.37 of dh-make.

@{GitaiInstallationPrefix = InstallationPrefix.replace('/opt/ros', '/opt/gitai/ros')}

# Uncomment this to turn on verbose mode.
export DH_VERBOSE=1
# TODO: remove the LDFLAGS override.  It's here to avoid esoteric problems
# of this sort:
#  https://code.ros.org/trac/ros/ticket/2977
#  https://code.ros.org/trac/ros/ticket/3842
export LDFLAGS=
export PKG_CONFIG_PATH=@(GitaiInstallationPrefix)/lib/pkgconfig:@(InstallationPrefix)/lib/pkgconfig
# Explicitly enable -DNDEBUG, see:
# 	https://github.com/ros-infrastructure/bloom/issues/327
export DEB_CXXFLAGS_MAINT_APPEND=-DNDEBUG
ifneq ($(filter nocheck,$(DEB_BUILD_OPTIONS)),)
	BUILD_TESTING_ARG=-DBUILD_TESTING=OFF -DCATKIN_ENABLE_TESTING=OFF
endif
# Support ccache
ifneq ($(shell which ccache),)
	BUILD_CCACHE_ARG=-DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache
endif
# Enable CATKIN_BUILD_BINARY_PACKAGE=1 for building package
# except for the `catkin` package to generate env files (i.e. setup.sh)
ifneq (@(Name),catkin)
	BUILD_BINARY_PACKAGE_ARG=-DCATKIN_BUILD_BINARY_PACKAGE="1"
endif

DEB_HOST_GNU_TYPE ?= $(shell dpkg-architecture -qDEB_HOST_GNU_TYPE)

%:
	dh $@@ -v --buildsystem=cmake --builddirectory=.obj-$(DEB_HOST_GNU_TYPE)

override_dh_auto_configure:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree that was dropped by catkin, and source it.  It will
	# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	if [ -f "@(GitaiInstallationPrefix)/setup.sh" ]; then . "@(GitaiInstallationPrefix)/setup.sh"; \
	elif [ -f "@(InstallationPrefix)/setup.sh" ]; then . "@(InstallationPrefix)/setup.sh"; fi && \
	dh_auto_configure -- \
		-DCMAKE_INSTALL_PREFIX="@(GitaiInstallationPrefix)" \
		-DCMAKE_PREFIX_PATH="@(GitaiInstallationPrefix);@(InstallationPrefix)" \
		$(BUILD_BINARY_PACKAGE_ARG) \
		$(BUILD_TESTING_ARG) \
		$(BUILD_CCACHE_ARG)

override_dh_auto_build:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree that was dropped by catkin, and source it.  It will
	# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	if [ -f "@(GitaiInstallationPrefix)/setup.sh" ]; then . "@(GitaiInstallationPrefix)/setup.sh"; \
	elif [ -f "@(InstallationPrefix)/setup.sh" ]; then . "@(InstallationPrefix)/setup.sh"; fi && \
	dh_auto_build

override_dh_auto_test:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree that was dropped by catkin, and source it.  It will
	# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	echo -- Running tests. Even if one of them fails the build is not canceled.
	if [ -f "@(GitaiInstallationPrefix)/setup.sh" ]; then . "@(GitaiInstallationPrefix)/setup.sh"; \
	elif [ -f "@(InstallationPrefix)/setup.sh" ]; then . "@(InstallationPrefix)/setup.sh"; fi && \
	dh_auto_test || true

override_dh_shlibdeps:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree that was dropped by catkin, and source it.  It will
	# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	if [ -f "@(GitaiInstallationPrefix)/setup.sh" ]; then . "@(GitaiInstallationPrefix)/setup.sh"; \
	elif [ -f "@(InstallationPrefix)/setup.sh" ]; then . "@(InstallationPrefix)/setup.sh"; fi && \
	dh_shlibdeps -l$(CURDIR)/debian/@(Package)@(GitaiInstallationPrefix)/lib/ || echo "Skip dh_shlibdeps error!!!"

override_dh_auto_install:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree that was dropped by catkin, and source it.  It will
	# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	if [ -f "@(GitaiInstallationPrefix)/setup.sh" ]; then . "@(GitaiInstallationPrefix)/setup.sh"; \
	elif [ -f "@(InstallationPrefix)/setup.sh" ]; then . "@(InstallationPrefix)/setup.sh"; fi && \
	dh_auto_install

override_dh_strip:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree that was dropped by catkin, and source it.  It will
	# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	if [ -f "@(GitaiInstallationPrefix)/setup.sh" ]; then . "@(GitaiInstallationPrefix)/setup.sh"; \
	elif [ -f "@(InstallationPrefix)/setup.sh" ]; then . "@(InstallationPrefix)/setup.sh"; fi && \
	dh_strip || true
