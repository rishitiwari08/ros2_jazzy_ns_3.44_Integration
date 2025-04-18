#!/bin/bash

set -e  # Exit if any command fails

NS_VERSION="ns-allinone-3.44"
NS_TAR="$NS_VERSION.tar.bz2"
NS_URL="https://www.nsnam.org/releases/$NS_TAR"

echo "Updating system packages..."
sudo apt update

echo "Installing required dependencies..."
sudo apt install -y g++ python3 cmake ninja-build git \
  gir1.2-goocanvas-2.0 python3-gi python3-gi-cairo python3-pygraphviz \
  gir1.2-gtk-3.0 ipython3 tcpdump wireshark sqlite3 libsqlite3-dev \
  qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools openmpi-bin \
  openmpi-common openmpi-doc libopenmpi-dev doxygen graphviz \
  imagemagick python3-sphinx dia texlive dvipng latexmk texlive-extra-utils \
  texlive-latex-extra texlive-font-utils libeigen3-dev gsl-bin \
  libgsl-dev libgslcblas0 libxml2 libxml2-dev libgtk-3-dev \
  lxc-utils lxc-templates vtun uml-utilities ebtables bridge-utils \
  libboost-all-dev ccache

echo "Downloading NS-3.44..."
cd ~
if [ ! -f "$NS_TAR" ]; then
  wget $NS_URL
fi

echo "Extracting NS-3.44..."
tar -xjf $NS_TAR

echo "Building NS-3.44..."
cd $NS_VERSION/
./build.py --enable-examples --enable-tests

echo "NS-3.44 installation completed successfully!"
