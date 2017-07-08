project = myproject
install_prefix ?= ~/ros_bin/$(project)


all:
	make -f Makefile.docu_sim_obstmove
#	cp docu_sim_obstmove ../../bin
#	cp docu_joy_to_cmd_setting.ini ../../bin

clean:
	make -f Makefile.docu_sim_obstmove clean
	
	
install:
	mkdir -p $(install_prefix)/lib/$(project)
	mkdir -p $(install_prefix)/share/$(project)/launch
	echo "<package><name>$(project)</name></package>" > $(install_prefix)/share/$(project)/package.xml
	touch $(install_prefix)/.catkin
	cp -a docu_sim_obstmove  $(install_prefix)/lib/$(project)
	
uninstall:
	rm -f $(install_prefix)/lib/$(project)/docu_sim_obstmove
	
	
	
