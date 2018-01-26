#ifndef LIDARMANAGER_H
#define LIDARMANAGER_H

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <array>
#include <math.h>
#include "rplidar.h"

typedef rplidar_response_measurement_node_t raw_node_t;

struct vec2_t {
	float x;
	float y;

	vec2_t(float x, float y) : x(x), y(y) { }
	vec2_t() : x(0), y(0) { }
};

float get_angle(raw_node_t & node);
float get_dst(raw_node_t & node);
vec2_t get_cartesian(raw_node_t & node, vec2_t & vec);

class Lidar {
    public:
        Lidar(std::string _comPath); 
        Lidar();
        ~Lidar();
        void start();
        void stop();
        void scan();
        float getAngle(int i);
        float getDist(int i);
    private:
        std::string comPath;
        std::array<raw_node_t, 360*2> nodes;
        _u32 baudrate = 115200;
        rp::standalone::rplidar::RPlidarDriver * drv;
        bool checkHealth();
};

#endif
