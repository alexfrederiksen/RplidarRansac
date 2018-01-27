#ifndef RANSAC_H
#define RANSAC_H

#include "lidarManager.h"
#include <stdlib.h>
#include <math.h>

namespace ransac {
    // defines regression lines in the format y = mx + b
    struct line_t {
        float m;
        float b;
        float get_x(float y);
        float get_y(float x);
    };

    struct node_t {
        float x;
        float y;
        float angle; // angle of raw data (in degrees)  
    };

    node_t & compute_raw_node(const raw_node_t & raw_node, node_t & out_node);
    
    class Ransac {
    private:
        int max_nodes;           // maximum number of nodes in a given run
        int max_trials;          // maximum number of trials that will be conducted
        int sample_size;         // number of samples to compute for each regression line
        float sample_deviation;  // the range of samples (in degrees) chosen around random one
        float proximity_epsilon; // proximity a point can be to a line      
        int line_consensus;      // number of points needed to confirm a line has been found

        node_t * temp_nodes = 0; // used to store a copy for computations

        std::vector<line_t> reg_lines; // contains confirmed regression lines

        int compute_reg_line(int start, int end, node_t nodes[], line_t & line);                // computes regression lines
        void restore_trial(node_t nodes[], int ref_index, int original_trial_size, int & size); // restores a trial to the starting state
        void pop_node(int node, node_t nodes[], int & size);                                    // pops a node out of the array
        float dst2_to_line(line_t & line, float x, float y);                                    // computes the square distance to line
    public:
        Ransac(int max_nodes, int max_trials, int sample_size, float sample_deviation, float proximity_epsilon, int line_consensus);
        ~Ransac();
        void compute(node_t nodes[], int size); // runs the RANSAC algorithm
        std::vector<line_t> get_reg_lines() { return reg_lines; }
    };
}

#endif
