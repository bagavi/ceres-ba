mkdir -p build
cd build
cmake ../test
make -j 4
./create_ba_data
