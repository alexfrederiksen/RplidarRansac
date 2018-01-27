#ifndef LIDARMANAGER_H
#define LIDARMANAGER_H

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <array>
#include <math.h>
#include "rplidar.h"

#define NODE_COUNT 360 * 2

typedef rplidar_response_measurement_node_t raw_node_t;

struct vec2_t {
    float x;
    float y;

    vec2_t(float x, float y) : x(x), y(y) { }
    vec2_t() : x(0), y(0) { }
};

float get_angle(const raw_node_t & node);
float get_dst(const raw_node_t & node);
vec2_t get_cartesian(const raw_node_t & node, vec2_t & vec);
void get_cartesian(const raw_node_t & node, float & x, float & y);

class Lidar {
    private:
        rp::standalone::rplidar::RPlidarDriver * drv;
        std::string com_path;
        raw_node_t nodes[NODE_COUNT];
        _u32 baudrate = 115200;

        bool check_health();
    public:
        Lidar(std::string _comPath); 
        Lidar();
        ~Lidar();
        void start();
        void stop();
        void scan();
        float getAngle(int i);
        float getDist(int i);
        
        raw_node_t * get_nodes() { return nodes; }
};

#endif
