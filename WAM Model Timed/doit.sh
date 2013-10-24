#!
rm -r build
mkdir build
cd build 
cmake ../
make
cd ..
#gazebo wam_world.sdf
