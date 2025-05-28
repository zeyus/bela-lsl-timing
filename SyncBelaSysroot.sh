#!/usr/bin/env bash
# Sync BelaSysroot with Bela board

BELA_IP=192.168.1.2

CURRENT_DIR=$(pwd)

if [ "$(id -u)" != "0" ]; then
    echo "This script must be run as root (files are required in /usr/local). Please use sudo."
    exit 1
fi

if [ -z "$BELA_IP" ]; then
    echo "Please set the BELA_IP variable to the IP address of your Bela board."
    exit 1
fi
if ! ping -c 1 $BELA_IP &> /dev/null; then
    echo "Bela board at $BELA_IP is not reachable. Please check the IP address and network connection."
    exit 1
fi

ssh root@$BELA_IP mkdir -p BelaRemote/Debug
ssh root@$BELA_IP mkdir -p BelaRemote/Release

echo "Creating directories on the local machine..."
mkdir -p /usr/local/linaro
chmod -R 755 /usr/local/linaro/

if [ -d /usr/local/linaro/arm-bela-linux-gnueabihf ]; then
    echo "arm-bela-linux-gnueabihf directory already exists. Please remove it if you want to download again."
else
    echo "Creating arm-bela-linux-gnueabihf directory..."
    cd /usr/local/linaro
    wget http://files.bela.io/gcc/arm-bela-linux-gnueabihf.zip
    unzip arm-bela-linux-gnueabihf.zip
    rm arm-bela-linux-gnueabihf.zip
    cd $CURRENT_DIR
fi


if [ -d ../Bela ]; then
    echo "Bela directory already exists. Please remove it if you want to clone again."
    
else
    echo "Cloning Bela platform repository..."
    git clone https://github.com/BelaPlatform/Bela.git ../Bela
fi


mkdir -p /usr/local/linaro/BelaSysroot/usr/xenomai/include
mkdir -p /usr/local/linaro/BelaSysroot/root/Bela/include
mkdir -p /usr/local/linaro/BelaSysroot/usr/include/alsa
mkdir -p /usr/local/linaro/BelaSysroot/usr/local/include

mkdir -p /usr/local/linaro/BelaSysroot/root/Bela/lib
mkdir -p /usr/local/linaro/BelaSysroot/usr/xenomai/lib
mkdir -p /usr/local/linaro/BelaSysroot/usr/local/lib
mkdir -p /usr/local/linaro/BelaSysroot/usr/lib/arm-linux-gnueabihf/

echo "Syncing BelaSysroot with Bela board at $BELA_IP..."

rsync -avz root@$BELA_IP:/usr/xenomai/include /usr/local/linaro/BelaSysroot/usr/xenomai
rsync -avz root@$BELA_IP:/usr/include/alsa /usr/local/linaro/BelaSysroot/usr/include
rsync -avz root@$BELA_IP:/root/Bela/include /usr/local/linaro/BelaSysroot/root/Bela
rsync -avz root@$BELA_IP:/root/Bela/build/pru/pru_rtaudio_irq_bin.h /usr/local/linaro/BelaSysroot/root/Bela/include
rsync -avz root@$BELA_IP:/root/Bela/build/pru/pru_rtaudio_bin.h /usr/local/linaro/BelaSysroot/root/Bela/include
rsync -avz root@$BELA_IP:/usr/local/include/prussdrv.h /usr/local/linaro/BelaSysroot/usr/local/include
rsync -avz root@$BELA_IP:/usr/local/include/seasocks /usr/local/linaro/BelaSysroot/usr/local/include
rsync -avz root@$BELA_IP:/root/Bela/libraries /usr/local/linaro/BelaSysroot/root/Bela/include/

 
rsync -avz root@$BELA_IP:/root/Bela/lib /usr/local/linaro/BelaSysroot/root/Bela
rsync -avz root@$BELA_IP:/usr/xenomai/lib /usr/local/linaro/BelaSysroot/usr/xenomai

rsync -avz root@$BELA_IP:/usr/local/lib/libpd.* /usr/local/linaro/BelaSysroot/usr/local/lib
rsync -avz root@$BELA_IP:/usr/local/lib/libseasocks.* /usr/local/linaro/BelaSysroot/usr/local/lib
rsync -avz root@$BELA_IP:/usr/local/lib/libprussdrv.* /usr/local/linaro/BelaSysroot/usr/local/lib

rsync -avz root@$BELA_IP:/usr/lib/arm-linux-gnueabihf/libsndfile.* /usr/local/linaro/BelaSysroot/usr/lib/arm-linux-gnueabihf/
rsync -avz root@$BELA_IP:/usr/lib/arm-linux-gnueabihf/libasound.* /usr/local/linaro/BelaSysroot/usr/lib/arm-linux-gnueabihf/

rsync -avz root@$BELA_IP:/usr/include/ne10 /usr/local/linaro/BelaSysroot/usr/include
rsync -avz root@$BELA_IP:/usr/include/math_neon.h /usr/local/linaro/BelaSysroot/usr/include

rsync -avz root@$BELA_IP:/usr/lib/libNE10.* /usr/local/linaro/BelaSysroot/usr/lib
rsync -avz root@$BELA_IP:/usr/lib/libmathneon.* /usr/local/linaro/BelaSysroot/usr/lib

rsync -avz root@$BELA_IP:/usr/local/include/libpd /usr/local/linaro/BelaSysroot/usr/local/include
rsync -avz root@$BELA_IP:/usr/local/lib/libpd.so* /usr/local/linaro/BelaSysroot/usr/local/lib
