#pragma once
#include <SDL2/SDL.h>
#include "lidarManager.h"
#include "ransac.h"


#define SCREEN_WIDTH  640    // screen width in pixels
#define SCREEN_HEIGHT 480    // screen height in pixels
#define VIEW_SCALE    1/1000 // scales millimeters to device coords
#define POINT_SIZE    10     // point size in pixels

class Core {
    private:
	    SDL_Rect point_rect; // reusable rectangle used for drawing points
	    Lidar lidar;
	    ransac::node_t computed_nodes[NODE_COUNT];
	    ransac::Ransac ransac;

        bool running;
        SDL_Surface * sDisplay;
        SDL_Window * window;
        SDL_Renderer * renderer;
	
        void world_to_device(float & x, float & y);             // converts world coords to device coords
	    void device_to_screen(float & x, float & y);            // converts device coords to screen coords
	    void draw_point(float x, float y);                      // draw point in device coords (-1 to 1)
	    void draw_line(float x1, float y1, float x2, float y2); // draw line in device coords (-1 to 1)
    public:
	    Core();
        ~Core() { SDL_Quit(); }
        int run();

        bool init();
        void on_event(SDL_Event & e);
        void update();
        void render();
};
