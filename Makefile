INCLUDES="./include"
#SDL=-D_REENTRANT -I/usr/include/SDL2 -lSDL2
LSDL=-lSDL2
OBJECTS = core.o lidarManager.o ransac.o lidar.a

lidar: core.o lidarManager.o ransac.o bin
	g++ -std=c++11 -o bin/lidar $(OBJECTS) -pthread $(LSDL)

core.o: core.cpp core.h
	g++ -std=c++11 -c core.cpp -I$(INCLUDES)

lidarManager.o: lidarManager.cpp lidarManager.h
	g++ -std=c++11 -c lidarManager.cpp -I$(INCLUDES)

ransac.o: ransac.cpp ransac.h
	g++ -std=c++11 -c ransac.cpp -I$(INCLUDES)

bin:
	mkdir bin

clean:
	rm -f *.o
	rm -rf bin
