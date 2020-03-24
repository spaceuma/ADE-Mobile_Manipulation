cur=$PWD
mkdir build
cd build
cmake -DCMAKE_CXX_FLAGS=-std=c++11 ..
make
cd $cur

echo "Compilation completed"
