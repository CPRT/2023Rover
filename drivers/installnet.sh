sudo make uninstall
sudo make clean
sudo make netdev
sudo make install
sudo modprobe pcan
sudo modprobe can
sudo modprobe vcan
sudo modprobe slcan
sudo modprobe peak_usb

sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0



#sudo ip link set can0 type can

#sudo ip link set can0 up type can tq 12 prop-seg 25 phase-seg1 25 phase-seg2 29 sjw 10 dtq 12 dprop-seg 6 dphase-seg1 2 dphase-seg2 7 dsjw 12 fd on  loopback on
