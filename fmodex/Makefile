
all: installed

TARBALL = build/fmodex44008.tar.gz
TARBALL_URL = http://rtabmap.googlecode.com/files/fmodex44008.tar.gz
SOURCE_DIR = build/fmodex44008
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	mkdir -p fmodex
	cp -r $(SOURCE_DIR)/include fmodex/.
	cp -r $(SOURCE_DIR)/lib fmodex/.
	touch installed

clean:
	-rm -rf src $(SOURCE_DIR) installed

wipe: clean
	-rm -rf build
