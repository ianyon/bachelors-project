# Bachelors Final Project
Implementation of Real-Time 3D Perception and Efficient Grasp Planning for Everyday Manipulation Tasks of Stueckler (2011) for Bachelor's final project

# Installation instructions

```bash
sudo apt-get install clang python-wstool ninja ccache binutils-gold
cd /usr/lib/ccache
sudo ln -s ../../bin/ccache clang++
sudo ln -s ../../bin/ccache clang

cd
wget https://cmake.org/files/v3.4/cmake-3.4.0-Linux-x86_64.sh --no-check-certificate
sudo chmod +x cmake-3.4.0-Linux-x86_64.sh
sudo mkdir /opt/cmake
sudo sh cmake-3.4.0-Linux-x86_64.sh --prefix=/opt/cmake

mkdir ~/catkin_ws
cd ~/catkin_ws
wstool init src
wstool set bachelors_final_project --git https://github.com/ianyon/bachelors-project.git
wstool update bachelors_final_project
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro hydro
rosdep install --from-paths . --ignore-src --rosdistro hydro -y
cd ..
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
