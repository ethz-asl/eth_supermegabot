## Create and setup your maplab workspace

You can also use maplab in a separate workspace, please refer to the
[repository](https://github.com/ethz-asl/maplab_summer) to explore all options
of maplab.

```
mkdir -p ~/maplab_ws/src
cd ~/maplab_ws
catkin init
catkin config --extend ~/catkin_ws/devel/
catkin config --merge-devel
catkin config -DCMAKE_BUILD_TYPE=Release
```
Pull in the source code.
```
cd src
git clone https://github.com/ethz-asl/maplab_summer --recursive
git clone -b summer https://github.com/ethz-asl/maplab_dependencies --recursive
```
Build the project (this is where you can go for a second nap)
```
cd ~/maplab_ws
catkin build maplab
```
Finally add your workspace to the .bashrc such that it is sourced every time you start a new
shell (terminal).
```
echo "source ~/maplab_ws/devel/setup.bash" >> ~/.bashrc
```