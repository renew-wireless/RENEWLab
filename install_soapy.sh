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
UHD=false

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -e|--extra)
	EXTRA_PACKAGES=true
    shift # past argument
    ;;
    -u|--uhd)
	UHD=true
    shift # past argument
    ;;
    -h|--help)
	echo Install SoapySDR on Ubuntu 16.04 or later.  Source files are left in new Soapy directory.
	echo Usage: only allowable arguments are --extra to install extra packages and --uhd to install USRP UHD driver.
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
	sudo apt-get install -y python-opengl python3-opengl python-pyqtgraph python3-pyqtgraph
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
	sudo apt-get install -y libboost-all-dev libusb-1.0-0-dev python-mako doxygen python-docutils cmake build-essential
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

cd ..

#rm -r $SOAPY_DIR

sudo ldconfig
export PYTHONPATH=/usr/local/lib/python3/dist-packages/:"${PYTHONPATH}"
#echo export PYTHONPATH=/usr/local/lib/python3/dist-packages/:"\${PYTHONPATH}" | sudo tee /etc/profile.d/python3path.sh
echo /usr/local/lib/python3/dist-packages/ | sudo tee /usr/lib/python3/dist-packages/SoapySDR.pth
