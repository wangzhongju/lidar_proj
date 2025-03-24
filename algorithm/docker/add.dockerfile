FROM harbor.ks.x/crayon/autoware-universe:v0.8.0.dev

RUN apt update && apt install ros-humble-gps-msgs && apt clean && rm -rf /var/lib/apt/lists/* && rm -rf /autoware && rm -rf /home/crayon 

