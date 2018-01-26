#pragma once
#include <SDL2/SDL.h>

#define SCREEN_WIDTH  640 // screen width in pixels
#define SCREEN_HEIGHT 480 // screen height in pixels
#define POINT_SIZE    10  // point size in pixels

class Core {
    public:
        Core();
        ~Core() { SDL_Quit(); }
        int run();

        bool init();
        void on_event(SDL_Event & e);
        void update();
        void render();
    private:
	SDL_Rect point_rect; // reusable rectangle used for drawing points

        bool running;
        SDL_Surface * sDisplay;
        SDL_Window * window;
        SDL_Renderer * renderer;
	
	void to_screen_coords(float & x, float & y);            // converts device points to screen coords
	void draw_point(float x, float y);                      // draw point in device coords (-1 to 1)
	void draw_line(float x1, float y1, float x2, float y2); // draw line in device coords (-1 to 1)
};
