sudo apt-get -y install libopencv-dev cmake libboost-all-dev

export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$PWD/deps/sherpa_tt_api/cmake/
install_folder=$PWD/install

export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$install_folder/lib/pkgconfig
cur=$PWD


## To install opencv follow https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html
## To install useful libraries for python visualization:
# sudo apt-get -y install python3-pip
# pip3 install mayavi
# pip3 install PyQt5
# sudo apt-get -y install python3-matplotlib

cd $cur/deps/sherpa_tt_api/base-logging
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder ..
make install

cd $cur/deps/sherpa_tt_api/base-types
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder ..
make install

cd $cur/deps/sherpa_tt_api/ip_tunnel
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder ..
make install

cd $cur/deps/sherpa_tt_api/detached_logger
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder ..
make install

cd $cur/deps/sherpa_tt_api/proxy_library
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder -DCMAKE_CXX_FLAGS=-std=c++11 ..
make install

cd $cur/deps/sherpa_tt_api/proxy_library_sherpa_tt
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder -DCMAKE_CXX_FLAGS=-std=c++11 ..
make install

cd $cur/spike/dummy_lib
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder -DCMAKE_CXX_FLAGS=-std=c++11 ..
make install

cd $cur/spike/plannerExample
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder -DCMAKE_CXX_FLAGS=-std=c++11 ..
make install

cd $cur/test/harnessExample
#rm -rf build
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder -DCMAKE_CXX_FLAGS=-std=c++11 ..
make

cd $cur/test/unit
#rm -rf build
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder -DCMAKE_CXX_FLAGS=-std=c++11 ..
make

cd $cur

echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$install_folder/lib" > env.sh
echo "export PATH=$PATH:$install_folder/bin" >> env.sh
echo "export PATH=$PATH:$cur/test/harnessExample/build" >> env.sh
echo "export PATH=$PATH:$cur/test/unit/build" >> env.sh

echo 
echo "Installation finished..."
echo
echo "Some Infos:"
echo "-----------"
echo "* To setup all paths run 'source env.sh'"
echo "* Libraries and headers are installed to $install_folder"
echo "* Have a look at the examples in 'proxy_library_sherpa_tt/examples'"
echo "* To try the examples execute the programs 'sherpa_tt_recv' and 'sherpa_tt_send'"
echo

