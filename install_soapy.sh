#!/bin/bash
#
#   Install SoapySDR and dependencies from latest source.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#   INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#   PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#   FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#   DEALINGS IN THE SOFTWARE.
#
#   (c) info@skylarkwireless.com 2018
#   SPDX-License-Identifier: BSD-3-Clause

SOAPY_DIR=Soapy
EXTRA_PACKAGES=false
TEST_PACKAGES=false
UHD=false
OSMO=false

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -e|--extra)
	EXTRA_PACKAGES=true
    shift # past argument
    ;;
    -t|--test)
	TEST_PACKAGES=true
    shift # past argument
    ;;
    -u|--uhd)
	UHD=true
    shift # past argument
    ;;
    -o|--osmo)
	OSMO=true
    shift # past argument
    ;;
    -h|--help)
	echo "Install SoapySDR on Ubuntu 16.04 or later.  Source files are left in new Soapy directory."
	echo "Usage: only allowable arguments are --extra (-e) to install extra packages, --osmo (-o) to install GrOsmoSDR, 
	--test (-t) to install packages needed for testing, and --uhd (-u) to install USRP UHD driver."
	exit 0
    shift # past argument
    ;;
    *)    # unknown option
	echo Unknown option $1
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done

if [[ -e $SOAPY_DIR ]] ; then
    echo Error, directory or file $SOAPY_DIR exists.  Delete and try again.
	exit -1
fi

#get dependencies
sudo apt update
sudo apt install -y software-properties-common git python3 python-numpy python3-numpy cmake swig python-dev build-essential libqt4-dev swig sip-dev python3-dev avahi-daemon libavahi-client-dev
sudo apt install -y python3-software-properties python-software-properties || true #these error on 18.04, so do them seperately. 

#remove existing install
sudo apt remove -y soapysdr soapysdr-server libsoapysdr-dev python-soapysdr python3-soapysdr soapysdr-module-remote || true

if [ "$EXTRA_PACKAGES" = true ] ; then
	echo Installing extra packages...
	sudo apt install -y python-opengl python3-opengl python-pyqtgraph python3-pyqtgraph python3-matplotlib python3-scipy python3-pyqt5 
fi

if [ "$TEST_PACKAGES" = true ] ; then
	echo Installing test packages...
	sudo apt install -y sshpass libudev-dev nlohmann-json-dev python3-coloredlogs python3-pil python3-termcolor python3-libusb1 net-tools ifupdown ethtool udev kmod tree net-tools u-boot-tools bash-completion bc tcpdump resolvconf openssl iputils-ping inetutils-syslogd avahi-daemon i2c-tools isc-dhcp-client rsync nginx lm-sensors screen libsystemd-dev libavahi-client-dev libi2c-dev bsdmainutils devscripts debhelper autoconf automake libyaml-cpp-dev libzmq3-dev libarmadillo-dev python3-pip
	sudo -q pip3 install aiohttp asyncssh async_generator async_exit_stack treelib argcomplete pyserial
fi

mkdir $SOAPY_DIR
cd $SOAPY_DIR

git clone https://github.com/pothosware/SoapySDR.git
cd SoapySDR
mkdir -p build
cd build
cmake ../
make -j`nproc`
sudo make install
cd ../..


git clone https://github.com/pothosware/SoapyRemote.git
cd SoapyRemote
mkdir -p build
cd build
cmake ../
make -j`nproc`
sudo make install
cd ../..

git clone https://github.com/skylarkwireless/sklk-soapyiris.git
cd sklk-soapyiris
mkdir -p build
cd build
cmake ../
make -j`nproc`
sudo make install
cd ../..

if [ "$UHD" = true ] ; then
	echo Installing UHD driver...
	sudo apt install -y libboost-all-dev libusb-1.0-0-dev python-mako doxygen python-docutils
	git clone git://github.com/EttusResearch/uhd.git
	cd uhd/host
	git submodule init
	git submodule update
	mkdir build
	cd build
	cmake ..
	make -j`nproc`
	sudo make install
	cd ../../..


	git clone https://github.com/pothosware/SoapyUHD.git
	cd SoapyUHD
	mkdir build
	cd build
	cmake ..
	make -j`nproc`
	sudo make install
	cd ../..
fi

if [ "$OSMO" = true ] ; then
	echo Installing GrOsmoSDR...
	sudo apt install -y gnuradio
	git clone git://git.osmocom.org/gr-osmosdr
	cd gr-osmosdr
	mkdir build
	cd build
	cmake ..
	make -j
	sudo make install
	cd ../..
	
	git clone https://github.com/pothosware/SoapyOsmo.git
	cd SoapyOsmo
	mkdir build
	cd build
	cmake ..
	make
	sudo make install
fi

cd ..

#rm -r $SOAPY_DIR

sudo ldconfig
export PYTHONPATH=/usr/local/lib/python3/dist-packages/:"${PYTHONPATH}"
#echo export PYTHONPATH=/usr/local/lib/python3/dist-packages/:"\${PYTHONPATH}" | sudo tee /etc/profile.d/python3path.sh
echo /usr/local/lib/python3/dist-packages/ | sudo tee /usr/lib/python3/dist-packages/SoapySDR.pth
