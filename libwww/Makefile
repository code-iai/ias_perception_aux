all: installed

INSTALL_DIR=$(shell pwd)/libwww
BUILD_DIR=$(shell pwd)/build
SOURCE_DIR=$(BUILD_DIR)/libwww/


checkedout:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && wget http://www.w3.org/Library/Distribution/w3c-libwww-5.4.0.tgz 
	cd $(BUILD_DIR) && tar -xvvf w3c-libwww-5.4.0.tgz && mv w3c-libwww-5.4.0 libwww
	touch checkedout

installed: wiped checkedout
	cd $(SOURCE_DIR) && ./configure --prefix=$(INSTALL_DIR)
	cd $(SOURCE_DIR) && make ${ROS_PARALLEL_JOBS} 
	cd $(SOURCE_DIR) && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	rm -rf $(BUILD_DIR) $(INSTALL_DIR)
	rm -f checkedout wiped installed

wipe: clean

