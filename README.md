# vehicle_sim_pygame
A 2d top down vehicle simulator using pygame library.

run with 
    `python3 vehicle_sim.py`

## python virtual env usin venv
To easily setup and run this python project use python's venv
Activate the python virtual environment with
    `source vehicle_sim/bin/activate`

The environment was created with 
    `python3 -m venv vehicle_sim`  

## Alternative virtualenv with pyenv virtualenv
mkvirtualenv -p $(pyenv which python) vehicle_sim

# Best virtual environment solution
use virtaulenv wrapper
  `pyenv global 3.7.3`      # Switch to python version 3.7.3
  `sudo pip install virtualenvwrapper` # install virtualenv wrapper
  `mkvirtualenv vehicle_sim`  # make the python virtual environment   
  `workon vehicle_sim`  # enter the virtual env
  `pip freeze > requirements.txt` # Create a text file of all the python libs
  `pip install -r requirements.txt` # install from that list

## Docker container
build with
    `vehicle_sim:latest`
run with  
    `runin.sh`

## pyp3
We can run using pypy3 instead of the standard cpython. This "may" run abit quicker.
Install pypy3 by downloading it and unpacking.
INstall pip for pypy3
`/home/calum/Downloads/pypy3.7-v7.3.5-linux64/bin/pypy3 -m ensurepip` 
Install dev pygame that works with pypy3
`~/Downloads/pypy3.7-v7.3.5-linux64/bin/pypy3 -m pip install 'Pygame>=1.9.4.dev0'`
Install numpy
`~/Downloads/pypy3.7-v7.3.5-linux64/bin/pypy3 -m pip install 'numpy'`
Run
`~/Downloads/pypy3.7-v7.3.5-linux64/bin/pypy3 vehicle_sim.py`