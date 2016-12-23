KERNEL_HEADERS=/lib/modules/$(shell uname -r)/build
KERNEL_MODULES=/lib/modules/$(shell uname -r)

VERBOSE=1

obj-m := nrf905.o

modules:
	@$(MAKE) -C $(KERNEL_HEADERS) M=$(PWD) modules

dtb:
	dtc -@ -I dts -O dtb -o nrf905.dtbo nrf905.dts
	
clean:
	@$(MAKE) -C $(KERNEL_HEADERS) M=$(PWD) clean
	rm -f nrf905.dtbo

modules_install:
	mkdir -p $(KERNEL_MODULES)/extra/
	cp nrf905.ko $(KERNEL_MODULES)/extra/

dtb_install:
	cp nrf905.dtbo /boot/overlays/
