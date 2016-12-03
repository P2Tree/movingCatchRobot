echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout", SYMLINK+="dobot"' > /etc/udev/rules.d/dobot.rules

service udev reload
sleep 2
service udev restart
