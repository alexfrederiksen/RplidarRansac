#include "core.h"

bool in_device_bounds(float value) {
    return value >= -1 && value <= 1;
}

bool in_device_bounds(vec2_t & vec) {
    return in_device_bounds(vec.x) &&
           in_device_bounds(vec.y);
}

void Core::update() {
    // perform a scan
    lidar.scan();
    // compute all the nodes
    raw_node_t * raw_nodes = lidar.get_nodes();
    for (int i = 0; i < NODE_COUNT; i++) {
        compute_raw_node(raw_nodes[i], computed_nodes[i]);
    }
    // throw at RANSAC algorithm
    ransac.compute(computed_nodes, NODE_COUNT);
}

void Core::render() {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    // render nodes
    for (int i = 0; i < NODE_COUNT; i++) {
        float x = computed_nodes[i].x;
        float y = computed_nodes[i].y;
        world_to_device(x, y);
        draw_point(x, y);
    }
    // render lines
    std::vector<ransac::line_t> lines = ransac.get_reg_lines();
    for (int i = 0; i < lines.size(); i++) {
        // compute endpoints from line
        ransac::line_t l;
        l.m = lines[i].m;              // already in device space
        l.b = lines[i].b * VIEW_SCALE; // convert to device space
        // define bounds of window in device coords
        vec2_t clips[] {
            vec2_t(-1, l.get_y(-1)), // left bound
            vec2_t(1, l.get_y(1)),   // right bound
            vec2_t(l.get_x(-1), -1), // bottom bound
            vec2_t(l.get_x(1), 1)    // top bound
        };
        // find all points that are bounded by device coords
        vec2_t * points[2];
        int point_count;
        for (int i = 0; i < 4; i++) {
            if (in_device_bounds(clips[i])) {
                    points[point_count] = &clips[i];
                    point_count++;
                    if (point_count >= 2) break;
            }
        }
        // check if we can actually draw it
        if (point_count >= 2) {
            // draw the line
            draw_line(points[0]->x, points[0]->y, points[1]->x, points[1]->y);
        }
    }
    // refresh screen
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

Core::Core() : ransac(100, 10, 2.0f, 1.0f, 10) {
    sDisplay = NULL;
    running = true;

    point_rect.w = POINT_SIZE;
    point_rect.h = POINT_SIZE;
}

bool Core::init() {
    if(SDL_Init(SDL_INIT_EVERYTHING) < 0) return false;
    // start up the lidar
    lidar.start();
    return SDL_CreateWindowAndRenderer(640, 480, 0, &window, &renderer) == 0;
}

/**
 * Converts coordinates from world space to device space [-1, 1]
 */ 
void Core::world_to_device(float & x, float & y) {
    x *= VIEW_SCALE;
    y *= VIEW_SCALE;
}

/*
 * Convert coordinates from device space [-1, 1] to screen space [0, DIMENSION].
 * In device space, (-1, -1) is the bottom left corner.
 */
void Core::device_to_screen(float & x, float & y) {
    x = 0.5f * (x + 1.0f) * SCREEN_WIDTH;
    y = 0.5f * (-y + 1.0f) * SCREEN_HEIGHT;
}

void Core::draw_point(float x, float y) {
    // convert to screen coords
    device_to_screen(x, y);
    // compute rectangle center
    point_rect.x = (int) (x - point_rect.w * 0.5f);
    point_rect.y = (int) (y - point_rect.h * 0.5f);
    // draw to screen
    SDL_RenderFillRect(renderer, &point_rect);
}

void Core::draw_line(float x1, float y1, float x2, float y2) {
    // convert to screen coords
    device_to_screen(x1, y1);
    device_to_screen(x2, y2);
    // draw to screen
    SDL_RenderDrawLine(renderer, (int) x1, (int) y1, (int) x2, (int) y2);
}

int main(int, char **) {
    Core core;
    return core.run();
}
