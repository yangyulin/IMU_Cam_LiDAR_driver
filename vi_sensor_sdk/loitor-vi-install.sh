create_udev_rules() {
	echo "" > loitor-vi.rules
	echo 'KERNEL=="*", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ACTION=="add", ATTR{idVendor}=="04b4", MODE="666" ' >> loitor-vi.rules
	echo 'KERNEL=="*", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ACTION=="remove" ' >> loitor-vi.rules
}

if [ `whoami` != 'root' ]; then
   echo "You have to be root to run this script"
   exit 1;
fi

create_udev_rules
cp loitor-vi.rules /etc/udev/rules.d/
