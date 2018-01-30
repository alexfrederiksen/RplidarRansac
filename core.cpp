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
    if (!testing_enabled) {
        lidar.scan();
        //printf("Found %d nodes.\n", lidar.get_node_count());
        // compute all the nodes
        raw_node_t * raw_nodes = lidar.get_nodes();
        good_node_count = lidar.get_node_count();
        pop_dumb_nodes(raw_nodes, good_node_count);
        for (int i = 0; i < good_node_count; i++) {
            compute_raw_node(raw_nodes[i], computed_nodes[i]);
        }
        // throw at RANSAC algorithm
        ransac.compute(computed_nodes, good_node_count);
    } else {
        // sort test data
        for (int i = testing_data_size - 1; i > 0; i--) {
            // select largest angle
            int select = 0;
            for (int j = 1; j <= i; j++) {
                if (testing_data[j].angle > testing_data[select].angle) {
                    select = j;
                }
            }
            // swap select with end
            ransac::node_t high = testing_data[i];
            testing_data[i] = testing_data[select];
            testing_data[select] = high;
        }
        // throw test data at RANSAC algorithm
        ransac.compute(testing_data, testing_data_size);
    }
}

void Core::pop_dumb_nodes(raw_node_t nodes[], int & size) {
    for (int i = size - 1; i >= 0; i--) {
        raw_node_t current = nodes[i];
        if (get_dst(current) < DUMB_NODE_THRESH) {
            // is a dumb node (by shifting other to the left)
            for (int j = i; i < size - 1; i++) {
                nodes[j] = nodes[j + 1];
            }
            nodes[size - 1] = current;
            size--;
        }
    }
}

void Core::render() {
    SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
    SDL_RenderClear(renderer);
    SDL_SetRenderDrawColor(renderer, 0, 200, 0, 255);
    draw_point(0.0f, 0.0f, POINT_SIZE * 10.0f);
    SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
    // render nodes
    if (!testing_enabled) {
        for (int i = 0; i < good_node_count; i++) {
            float x = computed_nodes[i].x;
            float y = computed_nodes[i].y;
            world_to_device(x, y);
            draw_point(x, y);
        }
    } else {
        for (int i = 0; i < testing_data_size; i++) {
            float x = testing_data[i].x;
            float y = testing_data[i].y;
            world_to_device(x, y);
            draw_point(x, y);
        }
    }
    // render lines
    SDL_SetRenderDrawColor(renderer, 200, 0, 0, 255);
    std::vector<ransac::line_t> lines = ransac.get_reg_lines();
    for (int i = 0; i < lines.size(); i++) {
        //std::cout << "Draw line: m " << lines[i].m << " b " << lines[i].b << std::endl; 
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
        int point_count = 0;
        for (int i = 0; i < 4; i++) {
            if (in_device_bounds(clips[i])) {
                    points[point_count] = &(clips[i]);
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
        SDL_Delay(UPDATE_INTERVAL);
    }
    return 0;
}

Core::Core(ransac::node_t * testing_data, int size) :
    testing_data(testing_data),
    testing_data_size(size),
    testing_enabled(true),
    ransac(size, 10, 10, 10.0f, 1.0f, 30) {
}

Core::Core() : 
    testing_enabled(false),
    ransac(MAX_NODE_COUNT, 1000, 7, 10.0f, 5.0f, 15) {

}


Core::~Core() {
    SDL_Quit();
    if (!testing_enabled) lidar.stop();
}

bool Core::init() {
    sDisplay = NULL;
    running = true;

    point_rect.w = POINT_SIZE;
    point_rect.h = POINT_SIZE;

    if(SDL_Init(SDL_INIT_EVERYTHING) < 0) return false;
    // start up the lidar
    if (!testing_enabled) {
        lidar.init();
        lidar.start();
    }
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

void Core::draw_point(float x, float y, float size) {
    // convert to screen coords
    device_to_screen(x, y);
    point_rect.w = size;
    point_rect.h = size;
    // compute rectangle center
    point_rect.x = (int) (x - point_rect.w * 0.5f);
    point_rect.y = (int) (y - point_rect.h * 0.5f);
    // draw to screen
    SDL_RenderFillRect(renderer, &point_rect);
}

void Core::draw_point(float x, float y) {
    draw_point(x, y, POINT_SIZE);
}

void Core::draw_line(float x1, float y1, float x2, float y2) {
    // convert to screen coords
    device_to_screen(x1, y1);
    device_to_screen(x2, y2);
    // draw to screen
    SDL_RenderDrawLine(renderer, (int) x1, (int) y1, (int) x2, (int) y2);
}

int main(int arg_count, char ** arg_values) {
    for (int i = 0; i < arg_count; i++) {
        std::cout << "ARG " << i << " : " << arg_values[i] << std::endl;   
    }

    // check test mode
    if (arg_count > 1) {
        // test mode
        const int test_count = 100;
        const int radius = 2000;
        ransac::node_t test_data[test_count];
        for (int i = 0; i < test_count; i++) {
            //float angle = (2 * M_PI / test_count) * i;
            //test_data[i].x = radius * cos(angle);
            //test_data[i].y = radius * sin(angle);
            //test_data[i].angle = angle * (180 / M_PI); 
            test_data[i].x = rand() % 2000 - 1000;
            test_data[i].y = 1000.0f;
            //test_data[i].y = rand() % 2000 - 1000;
            test_data[i].angle = rand() % 360;
            //std::cout << test_data[i].x << " " << test_data[i].y << " " << test_data[i].angle << std::endl;
        }

        Core core(test_data, test_count);
        return core.run();
    } else {
        // normal mode
        Core core;
        return core.run();
    }
}
