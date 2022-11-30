echo "*** Mobile Manipulation Unit Tests ***"

echo "Building..."
sudo rm -rf build
mkdir build
cd build
cmake ..
make

echo "Built!"
