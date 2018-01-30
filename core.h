#pragma once
#include <SDL2/SDL.h>
#include "lidarManager.h"
#include "ransac.h"


#define SCREEN_WIDTH     640          // screen width in pixels
#define SCREEN_HEIGHT    480          // screen height in pixels
#define UPDATE_INTERVAL  200         // scans in milleseconds
#define VIEW_SCALE       1.0f/500.0f // scales millimeters to device coords
#define POINT_SIZE       3            // point size in pixels
#define DUMB_NODE_THRESH 5.0f         // min distance allowed for a node


class Core {
    private:
        bool testing_enabled;
        ransac::node_t * testing_data;
        int testing_data_size;

        SDL_Rect point_rect; // reusable rectangle used for drawing points
        Lidar lidar;
        ransac::node_t computed_nodes[MAX_NODE_COUNT];
        int good_node_count;
        ransac::Ransac ransac;

        bool running;
        SDL_Surface * sDisplay;
        SDL_Window * window;
        SDL_Renderer * renderer;
   
        void pop_dumb_nodes(raw_node_t nodes[], int & size);    // pops out the dumb looking nodes      
        void world_to_device(float & x, float & y);             // converts world coords to device coords
        void device_to_screen(float & x, float & y);            // converts device coords to screen coords
        void draw_point(float x, float y);                      // draw point in device coords (-1 to 1)
        void draw_point(float x, float y, float size);          // draw point with given size
        void draw_line(float x1, float y1, float x2, float y2); // draw line in device coords (-1 to 1)
    public:
        Core(ransac::node_t * testing_data, int size);
        Core();
        ~Core();
        int run();

        bool init();
        void on_event(SDL_Event & e);
        void update();
        void render();
};
