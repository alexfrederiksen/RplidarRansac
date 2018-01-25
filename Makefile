INCLUDES="./include"
SDL=-D_REENTRANT -I/usr/include/SDL2 -lSDL2
LSDL=-lSDL2
OBJECTS = main.o lidarManager.o lidar.a

lidar: core.o bin
	g++ -std=c++11 -o bin/lidar core.o -pthread $(SDL)

core.o:core.cpp
	g++ -std=c++11 -c core.cpp -I$(INCLUDES)

bin:
	mkdir bin

clean:
	rm *.o
	rm -rf bin
