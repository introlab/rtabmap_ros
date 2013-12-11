# avpd rev. HEAD
INSTALL_DIR = rtabmap

all: installed

SVN_DIR = build/rtabmap-svn
SVN_URL = https://rtabmap.googlecode.com/svn/branches/pcl_integration/rtabmap
SVN_REVISION = -rHEAD
#SVN_PATCH = opencvVer.patch
include $(shell rospack find mk)/svn_checkout.mk

CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=RELEASE \
             -D BUILD_LIBS_ONLY=ON \
             -D ROS_BUILD=ON \
             -D CMAKE_INSTALL_PREFIX=`rospack find rtabmap_lib`/$(INSTALL_DIR)

installed: $(SVN_DIR) patched
	mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	touch Makefile

clean:
	-cd $(SVN_DIR)/build && make clean
	rm -rf $(INSTALL_DIR) installed

.PHONY : clean download
