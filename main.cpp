#include <iostream>
#include "lidarManager.h"
#include "ransac.h"

using namespace rplidar::algorithms;

void scan(Lidar & lidar, node_t computed_nodes[]) {
	lidar.scan();
	for (int i = 0; i < NODE_COUNT; i++) {
		// compute nodes from raw nodes
		compute_raw_node(lidar.get_nodes()[i], computed_nodes[i]);	
	}
}

int main() {
    // create lidar object
    Lidar lidar;
    // create computed node array
    node_t computed_nodes[NODE_COUNT];
    std::cout << "Press enter to start motor..." << std::endl;
    getchar();
    // start lidar
    lidar.start();
    // do a scan
    scan(lidar, computed_nodes);
    for (int i = 0; i < NODE_COUNT; i++) {
	std::cout << "x: " << computed_nodes[i].x << " y: " << computed_nodes[i].y << std::endl;
    }	

    
    //lidar.scan();
    //for(int i = 0; i < 100; i++) {
    //    std::cout << "Index: " << i << "  Angle: " << lidar.getAngle(i) << "  Dist: " << lidar.getDist(i) << std::endl;
    //}

    std::cout << "Motor started." << std::endl;
    std::cout << "Press enter to stop motor" << std::endl;
    getchar();
    lidar.stop();

    std::cout << "Press enter to end program." << std::endl;
    getchar();
    return 0;
}
