PKG_DIR=$(shell rospack find cop_bvw_class)/
INSTALL_DIR=$(PKG_DIR)/Recognosco/build

CMAKE_BINDINGS_ARGS = -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) \
                                                                                                                                                                                      
all: installed build_recog
	make -f MakefileROS
	
installed:
	rm -rf Recognosco
	git clone git://github.com/Janosch/Recognosco.git Recognosco
	wget http://ias.in.tum.de/~klank/6classes.bin
	mv 6classes.bin classifier.bin
	touch installed

build_recog:
	mkdir -p Recognosco/build
	cd Recognosco/build && cmake .. $(CMAKE_BINDINGS_ARGS) && make && make install
	
clean:
	rm -rf Recognosco
	installed	