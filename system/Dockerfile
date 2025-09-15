ARG FROM="archlinuxarm-base"
FROM ${FROM}

ADD files/etc/pacman.d /etc/pacman.d/
RUN pacman-key --init && pacman-key --populate

# upgrade downloaded image to latest state
RUN --mount=target=/var/cache/pacman/pkg,type=cache,rw=true \
  sed -i '/CheckSpace/d' /etc/pacman.conf \
  && sed -Ei "s/PRESETS=.+/PRESETS=('default')/" /etc/mkinitcpio.d/linux-aarch64.preset \
  && pacman -Syu --noconfirm

RUN --mount=target=/var/cache/pacman/pkg,type=cache,rw=true \
  pacman -Syu --noconfirm --needed \
    wget \
    vim \
    tmux \
    strace \
    htop \
    base-devel \
    git \
    clang \
    bazel \
    rsync \
    uboot-tools \
    dtc

RUN userdel -r alarm \
  # allow only ssh key auth and root user
  && sed -Ei 's/#?PermitRootLogin.+/PermitRootLogin prohibit-password/' /etc/ssh/sshd_config \
  && sed -Ei 's/#?PasswordAuthentication.+/PasswordAuthentication no/' /etc/ssh/sshd_config \
  # rpi4 is using mmcblk1
  && sed -i 's/mmcblk0/mmcblk1/g' "/etc/fstab" \
  && rm -f /boot/initramfs-linux-fallback.img

ADD files/ /

# compile Device Tree overlay for the board
ADD car-overlay.dts /boot/
RUN dtc -@ -I dts -O dtb /boot/car-overlay.dts > /boot/dtbs/car-overlay.dtbo \
  # compile u-boot script boot.txt into boot.scr
  && cd /boot \
  && ./mkscr
