echo "*** Arm reachability computer using DART ***"

echo "Building..."
sudo rm -rf build
mkdir build
cd build
cmake ..
make

echo "Built!"
