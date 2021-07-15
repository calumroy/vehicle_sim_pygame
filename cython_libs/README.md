# Create a cython library to run the vehicle model.
Thsi should significantly speed up the simulation

* Make sure your in the python virtual workspace. 
    `workon vehicle_sim`
* Install cython with this
    `python3 -m pip install cython`
* Build the cython .so library file.
    `python3 setup.py build_ext --inplace`
