#include "core.h"

void Core::update() {
	// do cool stuff here
}

void Core::render() {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    // render some pretty lines
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    draw_line(-1.0f, -1.0f, 1.0f, 1.0f);
    draw_line(-1.0f, 1.0f, 1.0f, -1.0f);
    // render a gorgeous point
    draw_point(0.0f, 0.0f);
    SDL_RenderPresent(renderer);
}

void Core::on_event(SDL_Event& e) {
    if(e.type == SDL_QUIT || e.type == SDL_KEYDOWN) {
        running = false;
    }
}

int Core::run() {
    if(init() == false) 
        return -1;

    SDL_Event event;

    while(running) {
        while(SDL_PollEvent(&event)) {
            on_event(event);
        }
        update();
        render();
    }
    return 0;
}

Core::Core() {
    sDisplay = NULL;
    running = true;

    point_rect.w = POINT_SIZE;
    point_rect.h = POINT_SIZE;
}

bool Core::init() {
    if(SDL_Init(SDL_INIT_EVERYTHING) < 0) 
        return false;

    return SDL_CreateWindowAndRenderer(640, 480, 0, &window, &renderer) == 0;
}

/*
 * Convert coordinates from device space [-1, 1] to screen space [0, DIMENSION].
 * In device space, (-1, -1) is the bottom left corner.
 */
void Core::to_screen_coords(float & x, float & y) {
	x = 0.5f * (x + 1.0f) * SCREEN_WIDTH;
	y = 0.5f * (-y + 1.0f) * SCREEN_HEIGHT;
}

void Core::draw_point(float x, float y) {
	// convert to screen coords
	to_screen_coords(x, y);
	// compute rectangle center
	point_rect.x = (int) (x - point_rect.w * 0.5f);
	point_rect.y = (int) (y - point_rect.h * 0.5f);
	// draw to screen
	SDL_RenderFillRect(renderer, &point_rect);
}

void Core::draw_line(float x1, float y1, float x2, float y2) {
	// convert to screen coords
	to_screen_coords(x1, y1);
	to_screen_coords(x2, y2);
	// draw to screen
	SDL_RenderDrawLine(renderer, (int) x1, (int) y1, (int) x2, (int) y2);
}

int main(int, char **) {
    Core core;
    return core.run();
}
