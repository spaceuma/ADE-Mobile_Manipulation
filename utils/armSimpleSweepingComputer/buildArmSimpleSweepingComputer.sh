echo "*** Arm simple sweeping computer ***"

echo "Building..."
sudo rm -rf build
mkdir build
cd build
cmake ..
make

echo "Built!"
