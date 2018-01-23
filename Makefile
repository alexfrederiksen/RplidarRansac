INCLUDES="./include"
OBJECTS = main.o lidarManager.o lidar.a

lidar:$(OBJECTS)
	g++ -std=c++11 -o bin/lidar $(OBJECTS) -pthread 

main.o:main.cpp
	g++ -std=c++11 -c main.cpp -I $(INCLUDES)

lidarManager.o:lidarManager.cpp
	g++ -std=c++11 -c lidarManager.cpp -I $(INCLUDES)

clean:
	rm *.o
