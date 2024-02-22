#!/bin/bash
set -ex
RES_PATH=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)

if [ "$UID" -eq 0 ]; then
	echo "Run as non-root"
	exit 1
fi

sudo cp "$RES_PATH/99-car.rules" /etc/udev/rules.d/99-car.rules
sudo cp "$RES_PATH/pwm-setup.service" /etc/systemd/system/
sudo systemctl enable pwm-setup
sudo usermod -a -G dialout "$USER"
