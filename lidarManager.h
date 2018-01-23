#include "rplidar.h"
#include <array>

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
        std::array<rplidar_response_measurement_node_t, 360*2> nodes;
        _u32 baudrate = 115200;
        rp::standalone::rplidar::RPlidarDriver * drv;
        bool checkHealth();
};
