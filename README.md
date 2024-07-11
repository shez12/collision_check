# collision_check
## Reference:
https://github.com/ompl/ompl
https://github.com/lyfkyle/pybullet_ompl


## Installation

'''
git clone https://github.com/shez12/collision_check.git
cd collision_check
git clone https://github.com/ompl/ompl.git
'''
go to https://github.com/ompl/ompl.git and install ALL the dependencies
'''
cd ompl
mkdir build/Release
cd build/Release
cmake ../.. -DPYTHON_EXECUTABLE=/usr/bin/python3.8 
make -j 4 update_bindings
make -j 4 # replace "4" with the number of cores on your machine
'''

export omple to python env
in terminal, type
'''
 export PYTHONPATH=/home/hanglok/Desktop/pybullet_ompl/ompl/py-bindings:$PYTHONPATH
'''
