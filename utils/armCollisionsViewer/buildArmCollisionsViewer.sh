echo "*** Collision detection viewer using DART ***"
echo "To change the initial arm configuration, edit the configuration.txt file"

echo "Building..."
sudo rm -rf build
mkdir build
cd build
cmake ..
make

echo "Built!"
