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
  pacman -Rnsc --noconfirm linux-aarch64 uboot-raspberrypi \
  && pacman -Syu --noconfirm --needed \
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
  linux-rpi \
  libcamera \
  python

RUN userdel -r alarm \
  # allow only ssh key auth and root user
  && sed -Ei 's/#?PermitRootLogin.+/PermitRootLogin prohibit-password/' /etc/ssh/sshd_config \
  && sed -Ei 's/#?PasswordAuthentication.+/PasswordAuthentication no/' /etc/ssh/sshd_config \
  && rm -f /boot/initramfs-linux-fallback.img

# needed in psutil dependency
RUN ln -sf /usr/bin/gcc /usr/bin/aarch64-linux-gnu-gcc

ADD files/ /
RUN git clone https://github.com/trnila/ros2_car /root/ros2_car
