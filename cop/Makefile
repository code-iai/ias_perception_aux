TIME = 26042011


cop_config: all_ver$(TIME) cop_config.ini pr2config.ini
	cp pr2config.ini ./resource
	cp cop_config.ini ./resource
	touch cop_config



all_ver$(TIME): Makefile germandelidb.tar.gz
	rm -f all_ver*
	rm -f cop_resourc*.tar.gz
	rm -rf u
	wget http://ias.in.tum.de/~klank/cop_resource_$(TIME).tar.gz
	tar -xzf  cop_resource_$(TIME).tar.gz
	tar -xzf germandelidb.tar.gz
	chmod -R a+w resource/
	touch all_ver$(TIME)
	rm -rf resource/data_big resource/germandeli_big resource/images_with_keypoints
	mv u/pangerci/germandelidb/* resource/
	rm -rf u

germandelidb.tar.gz:
	rm -f germandelidb.tar.gz
	wget http://ias.in.tum.de/~klank/germandelidb.tar.gz
	
clean:
	rm -rf resource cop_resourc*.tar.gz
	rm -f all_ver*
