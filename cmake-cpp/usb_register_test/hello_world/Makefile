KERNEL_SOURCE_TREE:=/lib/modules/$(shell uname -r)/build
PWD:=$(shell pwd)

obj-m:=hello_world.o

.PHONY: modules clean

modules:
	make -C $(KERNEL_SOURCE_TREE) M=$(PWD) modules
clean:
	make -C $(KERNEL_SOURCE_TREE )

