## run this command so that docker can open the socket
# xhost +

#docker run -it --rm -dt -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $PWD:/home -e DISPLAY=unix$DISPLAY --device /dev/snd pygame_vehicle_sim:latest python3 /home/app.py
# /bin/bash
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $PWD:/home vehicle_sim:latest /bin/bash #python3  /home/app.py
