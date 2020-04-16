sudo rmmod cryptocard_mod
sudo dmesg -C
make clean
make
echo "make successful"
sudo insmod cryptocard_mod.ko
echo "insmod sucessful"
dmesg
