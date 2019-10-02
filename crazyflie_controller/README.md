## HOWTO

* Compile acados

```
git submodule update --init --recursive
pushd ../acados
make -j4
popd
```

* Create and activate a python virtualenv with python > 3.5 < 3.7
```
pip install ../acados/interfaces/acados_template
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$(pwd)/../acados/lib
python scritps/crazyflie_full_model/generate_c_code.py
```

* Run Catkin make
