#include "core.h"

void Core::loop() {
}

void Core::render() {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);
}

void Core::event(SDL_Event& e) {
    if(e.type == SDL_QUIT) {
        running = false;
    }
}

int Core::run() {
    if(init() == false) 
        return -1;

    SDL_Event e;

    while(running) {
        while(SDL_PollEvent(&e)) {
            event(e);
        }
        loop();
        render();
    }
    return 0;
}

Core::Core() {
    sDisplay = NULL;
    running = true;
}

bool Core::init() {
    if(SDL_Init(SDL_INIT_EVERYTHING) < 0) 
        return false;

    return SDL_CreateWindowAndRenderer(640, 480, 0, &window, &renderer) == 0;
}


int main(int, char**) {
    Core core;
    core.run();
}
