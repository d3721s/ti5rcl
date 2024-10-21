sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
sudo ip link set vcan0 type can bitrate 1000000
sudo ip link set vcan0 up
