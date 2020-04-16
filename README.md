# Crypto Driver
This is done as part for Advanced operating systems course. This project contains a driver code to interact with the crypto device and a library handle that can be used in userspace programs to derive the functionalities provided.

## Crypto Device
The crypto device provides hardware encrytion and and decryption using two 8bit keys. The device supports both mmio and dma.

## Driver program
The driver allows us to set the configuration in which we want to use the device. Once this is done we can invoke encrypt or decrypt functions. The driver is configurable in both mmio and dma setting with/without interrupts. It aslo allows multi-processing.

## library
**1.create_handle** It returns handle to driver code which is implemented as a character device.

**2.encrypt and decrypt** given data encrypt and decrypt it respectively.

**3.set_key** set the key.

**4.set_config** set the configuration in which u wnat to run the program.

**5.close_handle** Close the handle to character device.

## compile and install driver
```sh
cd drivers
make
sudo insmod crypto_mod.ko
```
## uninstall the driver
```sh
sudo rmmod crypto_mod
```

## compile library
```sh
cd .. # come back to root directory
make
```
