# avpd rev. HEAD
INSTALL_DIR = utilite

all: installed

SVN_DIR = build/utilite-svn
SVN_URL = https://utilite.googlecode.com/svn/trunk/
SVN_REVISION = -r173
#SVN_PATCH = opencvVer.patch
include $(shell rospack find mk)/svn_checkout.mk

CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=RELEASE \
             -D BUILD_AUDIO=ON \
             -D BUILD_QT=ON \
             -D BUILD_EXAMPLES=OFF \
             -D BUILD_TESTS=OFF \
             -D CMAKE_INSTALL_PREFIX=`rospack find utilite`/$(INSTALL_DIR)

installed: $(SVN_DIR) patched
	mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	touch Makefile

clean:
	-cd $(SVN_DIR)/build && make clean
	rm -rf $(INSTALL_DIR) installed

.PHONY : clean download
