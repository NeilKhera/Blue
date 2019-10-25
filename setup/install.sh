# Install udev rules
sudo cp ./10-blue.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
