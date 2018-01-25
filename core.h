#pragma once

#include <SDL2/SDL.h>

class Core {
    public:
        Core();
        ~Core() { SDL_Quit(); }
        int run();
    public:
        bool init();
        void event(SDL_Event& e);
        void loop();
        void render();
    private:
        bool running;
        SDL_Surface *sDisplay;
        SDL_Window *window;
        SDL_Renderer *renderer;
};
