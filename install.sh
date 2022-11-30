sudo apt-get -y install libopencv-dev cmake libboost-all-dev

# Install DART
#sudo apt-add-repository ppa:dartsim/ppa
#sudo apt-get update  # not necessary since Bionic
sudo apt-get -y install libdart-all-dev

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

## To install google test
#sudo apt install libgtest-dev build-essential cmake
#cd /usr/src/googletest
#sudo cmake .
#sudo cmake --build . --target install

cd $cur
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder -DCMAKE_CXX_FLAGS=-std=c++11 ..
make

cd $cur/data/planner
rm urdfmodel_path.txt
echo "$cur/data/planner" >> urdfmodel_path.txt
mkdir results


#cd $cur/test/harnessExample
##rm -rf build
#mkdir build
#cd build
#cmake -DCMAKE_INSTALL_PREFIX=$install_folder -DCMAKE_CXX_FLAGS=-std=c++11 ..
#make

cd $cur/test/unit
#rm -rf build
mkdir data/results
mkdir data/results/MMMapTest
mkdir data/results/MMMotionPlanTest
mkdir data/results/MMExecutorTest
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder -DCMAKE_CXX_FLAGS=-std=c++11 ..
make

cd $cur/utils/armCollisionsViewer
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder ..
make

cd $cur/utils/armReachabilityComputer
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder ..
make

cd $cur/utils/armSimpleSweepingComputer
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder ..
make

cd $cur


echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$install_folder/lib" > env.sh
echo "export PATH=$PATH:$install_folder/bin:$cur/test/harnessExample/build:$cur/test/unit/build" >> env.sh

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

