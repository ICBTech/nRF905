# Linux driver for the Nordic nRF905 transceiver


## Introduction

The driver provides a character device and a few sysfs attributes, allowing for simple wireless communication with another Raspberry Pi or perhaps an Arduino with the same nRF905 chip.


### Character device

`/dev/nrf905`

Allowed operations are write and read. The file can be opened for both read and write operations (`r+` mode, see `man 3 fopen` for further details).

Read and write operations only work with 32 bytes at a time (the size of the buffer on the nRF905 chip). If a different size is used, `EMSGSIZE` is returned.


### Sysfs attributes

`/sys/bus/spi/devices/spi0.0/frequency`

The frequency (in kHz) that the transceiver is tuned to. Allowed ranges are [422400, 473600) and [844800, 947200), with rounding to 100kHz in the former and 200kHz in the latter case. The lower bound is included in both cases, the upper is excluded. Both `HFREQ_PLL` and `CH_NO` are set based on the frequency written to this sysfs attribute.


`/sys/bus/spi/devices/spi0.0/pa_pwr`

Output power. Allowed values are from 0 to 3 (both inclusive).

*   0: -10dBm
*   1: -2dBm
*   2: +6dBm
*   3: +10dBm


`/sys/bus/spi/devices/spi0.0/rx_address`

The address used when receiving. At the moment, the driver supports only 4-byte addresses.


`/sys/bus/spi/devices/spi0.0/tx_address`

The address used when transmitting. At the moment, the driver supports only 4-byte addresses.


## Simple usage

### General configuration

This needs to be done only once per boot.

```
sudo sh -c "echo -n ABCD > /sys/bus/spi/devices/spi0.0/rx_address"
sudo sh -c "echo -n ABCD > /sys/bus/spi/devices/spi0.0/tx_address"
sudo sh -c "echo -n 868000 > /sys/bus/spi/devices/spi0.0/frequency"
sudo sh -c "echo -n 3 > /sys/bus/spi/devices/spi0.0/pa_pwr"
```

### Receiver

```
sudo dd if=/dev/nrf905 bs=32 count=1 2>/dev/null | hexdump -Cv
```

### Transmitter

```
sudo dd if=/dev/urandom bs=32 count=1 of=/dev/nrf905
```

For normal usage, you should change the permissions (see `man 1 chmod`), but that is out of the scope of this document.

## Installation instructions

### Install Raspbian Jessie

At the moment of writing, the current version is `2016-11-25` and the lite edition has been used while writing this document.

See <https://www.raspberrypi.org/documentation/installation/installing-images/README.md> for instructions on how to make a bootable SD card.


### Update the system

```sh
sudo apt-get update
sudo apt-get dist-upgrade
```


### Install a kernel with exported symbols

```sh
sudo apt-get install linux-image-4.4.0-1-rpi2 linux-image-rpi2-rpfv linux-headers-rpi2-rpfv
```

Note: Use `rpi` instead of `rpi2` if you have the original Raspberry Pi.


### Set the default kernel

Edit `/boot/config.txt` and add the following lines

```
# use the rpfv kernel
kernel=vmlinuz-4.4.0-1-rpi2
initrd=initrd.img-4.4.0-1-rpi2 followkernel
```

Note: Use `rpi` instead of `rpi2` if you have the original Raspberry Pi.


### Boot with the new kernel

```sh
sudo reboot
```


### Verify the current kernel

```sh
uname -r
```

If everything is correct, the output should be

```
4.4.0-1-rpi2
```


### Clone this repository

```sh
git clone git://this.repo
```


### Build the module against the currently running kernel

```sh
make
make dtb
```


### Install the module and overlay

```sh
sudo make modules_install
sudo make dtb_install
sudo depmod -a
```


### Manually load the module

```sh
sudo modprobe nrf905
```

### Check that the module has been loaded

```sh
dmesg | grep nrf905
```

If everything is correct, the output should be

```
[   79.829085] nrf905_init
```

The timestamp would, of course, be different.


### Enable the overlay

Edit `/boot/config.txt` and add the following lines

```
# load our custom overlay
dtoverlay=nrf905
```

In that same file, also uncomment the line to enable spi

```
# enable spidev
dtparam=spi=on
```


```sh
sudo reboot
```


### Manually load the module

```sh
sudo modprobe nrf905
```

### Check that the module has been loaded

```sh
dmesg | grep nrf905
```

If everything is correct, the output should be

```
[   25.095296] nrf905_init
[   25.096562] nrf905 spi0.0: nrf905_probe
[   25.096615] nrf905 spi0.0: gpio_pwr_up: 17
[   25.096627] nrf905 spi0.0: gpio_trx_ce: 27
[   25.096638] nrf905 spi0.0: gpio_tx_en: 22
[   25.096649] nrf905 spi0.0: gpio_dr: 18
```

The timestamps would, of course, be different.
Compared to the previous run of the same command, the output now also contains lines where the driver is probed (bound to the spi address defined in the dts file).


### Automatically load the module at boot

Edit `/etc/modules` and add the following line

```
nrf905
```
