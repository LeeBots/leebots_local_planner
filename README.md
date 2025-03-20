## 0. install and run the-barn-challenge pkg

```bash
# make env
pip3 install defusedxml rospkg netifaces numpy
cd ~/barn_ws/
mkdir jackal_ws/src
cd jackal_ws/src

# download pakages
git clone https://github.com/Daffan/the-barn-challenge.git
git clone https://github.com/jackal/jackal.git --branch noetic-devel
git clone https://github.com/jackal/jackal_simulator.git --branch noetic-devel
git clone https://github.com/jackal/jackal_simulator.git --branch melodic-devel
git clone https://github.com/jackal/jackal_desktop.git --branch noetic-devel
git clone https://github.com/jackal/jackal_desktop.git --branch melodic-devel
git clone https://github.com/utexas-bwi/eband_local_planner.git

# build
cd ..
source /opt/ros/noetic/setup.bash
rosdep init; rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro=noetic
pip install empy==3.3.4
# cmake_err: changing -std=c++11 to -std=c++17 in jackal_helper/CMakeLists.txt line 3
catkin_make

# run simulation

source ../../devel/setup.sh
python3 run.py --world_idx 0
```
## 1. install leebots_local_planner
```bash
 $cd ~/barn_ws/jackal_ws/src/leebots_local_planner
 $git clone https://github.com/LeeBots/leebots_local_planner.git
```

## 2. Train leebots_local_planner
```bash
source ../../../devel/setup.sh
python3 train_leebots_td3.py
```

#### reference code
```bash
https://github.com/reiniscimurs/DRL-robot-navigation/blob/main/TD3/velodyne_env.py
https://github.com/reiniscimurs/DRL-robot-navigation/blob/main/TD3/train_velodyne_td3.py
https://github.com/reiniscimurs/DRL-robot-navigation/blob/main/TD3/replay_buffer.py
https://github.com/Daffan/the-barn-challenge/blob/main/run.py
```
