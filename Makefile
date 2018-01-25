INCLUDES="./include"
OBJECTS = main.o lidarManager.o ransac.o lidar.a

lidar:$(OBJECTS) bin
	g++ -std=c++11 -o bin/lidar $(OBJECTS) -pthread 

main.o:main.cpp
	g++ -std=c++11 -c main.cpp -I $(INCLUDES)

lidarManager.o:lidarManager.cpp
	g++ -std=c++11 -c lidarManager.cpp -I $(INCLUDES)

ransac.o: ransac.cpp
	g++ -std=c++11 -c ransac.cpp -I $(INCLUDES)

bin:
	mkdir bin

clean:
	rm -f *.o
	rm -f -r bin
