#include "lidarManager.h"

using namespace rp::standalone::rplidar;

float get_angle(const raw_node_t & node) {
    return (node.angle_q6_checkbit >> 1) / 64.0f;
}

float get_dst(const raw_node_t & node) {
    return node.distance_q2 / 4.0f;
}

vec2_t get_cartesian(const raw_node_t & node, vec2_t & vec) {
	get_cartesian(node, vec.x, vec.y);
	return vec;
}

void get_cartesian(const raw_node_t & node, float & x, float & y) {
	float dst = get_dst(node);
	float angle_rad = get_angle(node) * (M_PI / 90.0f); 
	x = dst * cos(angle_rad);
	y = dst * sin(angle_rad);
}

// Constructors
Lidar::Lidar(std::string com_path) 
    : com_path(com_path)
    , drv(RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT)) {
        if(!drv) {
            std::cerr << "insufficient memory, bye" << com_path << std::endl;
            throw "insufficient memory";
        }
        //connect
        if(IS_FAIL(drv->connect(com_path.c_str(), baudrate))) {
            std::cerr << "Error: Cannot bind to serial port " << com_path << std::endl;
            throw "can't bind it";
        }

        if(!check_health()) {
            throw "unhealth";
        }

}

Lidar::Lidar() : Lidar("/dev/ttyUSB0") { }

Lidar::~Lidar() {
    stop();
    RPlidarDriver::DisposeDriver(drv);
}

/////////////////////////////////////////////

void Lidar::start() {
    drv->startMotor();
    drv->startScan();
}

void Lidar::stop() {
    drv->stop();
    drv->stopMotor();
}

void Lidar::scan() {
    //raw_node_t _nodes[360*2];
    size_t nodeSize;
    
    drv->grabScanData(nodes, nodeSize);
    std::cout << "NodeCount: " << nodeSize << std::endl;
    //std::copy(std::begin(_nodes), std::end(_nodes), std::begin(nodes));
    
}

float Lidar::getAngle(int i) {
    return (nodes[i].angle_q6_checkbit >> 1) / 64.0f;
}

float Lidar::getDist(int i) {
    return nodes[i].distance_q2/4.0f;
}

bool Lidar::check_health() {
    u_result result;
    rplidar_response_device_health_t healthInfo;

    result = drv->getHealth(healthInfo);
    if(IS_OK(result)) {
        std::cout << "Health: " << healthInfo.status << std::endl;
        if(healthInfo.status == RPLIDAR_STATUS_ERROR) {
            std::cerr << "Error: RPlidar unhealthy. Reboot and try again." << std::endl;
            return false;
        } else {
            return true;
        }
    } else {
        std::cerr << "Error: Cannot retrieve lidar health" << std::endl;
        return false;
    }
}


