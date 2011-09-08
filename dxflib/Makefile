
BUILD_DIR=dxflib-src
DXF_PREFIX=`rospack find dxflib`/dxflib

all: dxflib-src/Makefile
	cd $(BUILD_DIR) && make && make install

dxflib-src/Makefile:
	cd $(BUILD_DIR) && ./configure --prefix=$(DXF_PREFIX)

