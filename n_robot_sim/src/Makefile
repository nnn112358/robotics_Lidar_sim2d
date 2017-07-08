project = myproject
install_prefix ?= ~/ros_bin/$(project)
name=n_robot_sim

all:
	make -f Makefile.$(name)

clean:
	make -f Makefile.$(name) clean
	
	
install:
	mkdir -p $(install_prefix)/lib/$(project)
	mkdir -p $(install_prefix)/share/$(project)/launch
	echo "<package><name>$(project)</name></package>" > $(install_prefix)/share/$(project)/package.xml
	touch $(install_prefix)/.catkin
	cp -a docu_joy_to_cmd  $(install_prefix)/lib/$(project)
	
uninstall:
	rm -f $(install_prefix)/lib/$(project)/$(name)
	
	
	
