# collision_check

## Installation

### Clone the Repository

```bash
git clone https://github.com/shez12/collision_check.git
cd collision_check
git clone https://github.com/ompl/ompl.git

Visit OMPL GitHub Repository and install dependencies

```bash
cd ompl
mkdir build/Release
cd build/Release
cmake ../.. -DPYTHON_EXECUTABLE=/usr/bin/python3.8 # your python address
make -j 4 update_bindings 
make -j 4 # Replace "4" with the number of cores on your machine



```bash
# Export OMPL Python bindings to Python environment
export PYTHONPATH=/home/hanglok/Desktop/pybullet_ompl/ompl/py-bindings:$PYTHONPATH




## Reference
- [OMPL GitHub Repository](https://github.com/ompl/ompl)
- [pybullet_ompl GitHub Repository](https://github.com/lyfkyle/pybullet_ompl)
