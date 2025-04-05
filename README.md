To build with g++:

```g++ -pthread -o lasercam  main.cpp HeliosDac.cpp -lglfw -lGLU -lGL -lXrandr -lXxf86vm -lXi -lXinerama -lX11 -lrt -ldl -std=c++17 -lusb-1.0 -lrealsense2```

Make sure to install all necessary packages beforehand ;) 
