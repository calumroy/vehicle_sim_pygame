## run this command so that docker can open the socket
# xhost +

#docker run -it --rm -dt -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $PWD:/home -e DISPLAY=unix$DISPLAY --device /dev/snd pygame_vehicle_sim:latest python3 /home/app.py
# /bin/bash
#docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $PWD:/home vehicle_sim:latest /bin/bash #python3  /home/app.py
#docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix:rw --privileged -e DISPLAY=unix$DISPLAY -v $PWD:/home vehicle_sim:latest /bin/bash #python3  /home/app.py
docker run -it -e DISPLAY=unix$DISPLAY --device /dev/nvidia0:/dev/nvidia0 --device /dev/nvidiactl:/dev/nvidiactl --device /dev/nvidia-uvm:/dev/nvidia-uvm -v $PWD:/home vehicle_sim:latest /bin/bash #python3  /home/app.py