#include "ransac.h"
#include <cstdarg>

namespace ransac {

    bool debug_enabled = false;
    int debug_level = 1;

    void debug(std::string msg, int level = 0) {
        if (debug_enabled) {
            std::cout << msg.c_str() << std::endl;
        }
    }

    void debugf(std::string format, ...) {
        if (debug_enabled) {
            va_list args;
            va_start(args, format);
            vprintf(format.c_str(), args);
            va_end(args);
        }
    }



    void print_nodes(node_t nodes[], int size, int capacity) {
        std::cout << "---[Node list]---:" << std::endl;
        for (int i = 0; i < size; i++) {
            std::cout << "Node " << nodes[i].angle << std::endl;
        }
        std::cout << "---[Popped nodes]---: " << std::endl;
        for (int i = size; i < capacity; i++) {
            std::cout << "Node " << nodes[i].angle << std::endl;
        }
    }

    float line_t::get_x(float y) {
        if (m == 0) {
            // cannot divide by zero
            return 0;
        }
        return (y - b) / m;
    }

    float line_t::get_y(float x) {
        return m * x + b;
    }

    /**
     * Computes a "node_t" from a given "raw_node_t"
     */
    node_t & compute_raw_node(const raw_node_t & raw_node, node_t & out_node) {
        // compute the cartesian coordinates for the raw node
        get_cartesian(raw_node, out_node.x, out_node.y);
        // compute the angle
        out_node.angle = get_angle(raw_node);
        return out_node;
    }
    
    Ransac::Ransac(int max_nodes, int max_trials, int sample_size, float sample_deviation, float proximity_epsilon, int line_consensus) :
        max_trials(max_trials), sample_size(sample_size),
        sample_deviation(sample_deviation), proximity_epsilon(proximity_epsilon),
        line_consensus(line_consensus) {
        // allocate temp array
        temp_nodes = new node_t[max_nodes];
    }

    Ransac::~Ransac() {
        // free memory
        delete [] temp_nodes;
    }


    /*
     * Computes regression lines based upon the raw data. Note this function assumes that the data
     * is pre-sorted by the angles (which natively it is (; ).
     */
    void Ransac::compute(node_t nodes[], int size) {
        // clear out the line array
        reg_lines.clear();
        int original_size = size; // size at the beginning of computation
        int trial_count = 0;
        debugf("Starting %d trials...\n", max_trials);
        //debug("Starting trials...");
        while (size > 0 && trial_count < max_trials) {
            int original_trial_size = size; // size at the beginning of each trial
            // choose a random reference node
            int ref_index = rand() % size;
            float ref_angle = nodes[ref_index].angle;
            debugf("Choosing reference index %d by random...\n", ref_index);
            // choose surrounding nodes within parameter
            for (int i = 0; i < sample_size; i++) {
                // alternate from left and right
                int pick;
                if (i % 2 == 0) {
                    // pick from left
                    pick = ((ref_index - 1) + size) % size;
                } else {
                    // pick from right
                    pick = (ref_index + 1) % size;
                }
                debugf("Looking at index %d next to reference...\n", pick);
                // validate that pick is within given deviation
                if (pick != ref_index && fabs(nodes[pick].angle - ref_angle) <= sample_deviation) {
                    // pick the node
                    debugf("Popping node %d after picking...\n", pick);
                    pop_node(pick, nodes, size);
                    // correct reference node position when popping from the left
                    if (pick < ref_index) ref_index--;
                }
            }
            // pop the reference node
            debugf("Popping reference node (at %d)...\n", ref_index);
            pop_node(ref_index, nodes, size);
            // compute least squares regression line
            debug("Computing raw regression line from initial group...");
            int sample_start = size;              // inclusive
            int sample_end = original_trial_size; // exclusive
            line_t line;
            int line_status = compute_reg_line(sample_start, sample_end, nodes, line);
            if (line_status == 0) {
                // associate other nodes with the line (by popping them)
                debug("Associating other nodes to the raw regression line...");
                for (int i = 0; i < size; i++) {
                    float dst2 = dst2_to_line(line, nodes[i].x, nodes[i].y);
                    if (dst2 <= proximity_epsilon * proximity_epsilon) {
                        // node is close enough to line
                        pop_node(i, nodes, size);
                        i--;    
                    }
                }
            } else {
                debug("Raw regression line failed to compute.");
            }
            // confirm line by given consensus count
            int popped_nodes = original_trial_size - size;
            debugf("%d nodes associated to the raw line.\n", popped_nodes);
            if (popped_nodes >= line_consensus && line_status == 0) {
                // recompute the regression line with all associated nodes
                debug("Computing new regression line for nodes...");
                line_status = compute_reg_line(size, original_trial_size, nodes, line);
                if (line_status == 0) { 
                    // push into line array
                    debug("Pushing new regression line...");
                    reg_lines.push_back(line);
                }
            } else {
                // line failure
                debug("New line failed to be created (did not meet consensus or failed before).");
                line_status = -1;
            }
            if (line_status != 0) {
                // put nodes back into trial array
                debug("Trial failed. Restoring the trial back...");
                restore_trial(nodes, ref_index, original_trial_size, size);
            }
            // increment the trial counter
            trial_count++;
            debug("Finished trial.");
        }
        debug("Finished RANSAC.");
    }


    /*
     *  Computes a regression line from the nodes in an array.
     *  returns the status code (0 = success, -1 = failure)
     */ 
    int Ransac::compute_reg_line(int start, int end, node_t nodes[], line_t & line) {
        int node_count = start - end;
        // return if no nodes where popped
        if (node_count == 0) return -1;
        // compute sums for regression line
        float x_sum = 0;
        float y_sum = 0;
        float x2_sum = 0;
        float xy_sum = 0; 
        for (int i = start; i < end; i++) {
            node_t n = nodes[i];
            x_sum += n.x;
            y_sum += n.y;
            x2_sum += n.x * n.x;
            xy_sum += n.x * n.y;
        }
        // compute slope
        float m_num = (node_count * xy_sum - x_sum * y_sum); 
        float m_denom = (node_count * x2_sum - x_sum * x_sum);
        if (m_denom == 0.0f) return -1;
        line.m = m_num / m_denom;
        //printf("Computed slope (num %f denom %f = %f) \n", m_num, m_denom, line.m); 
        // compute y-intercept
        line.b = (y_sum - line.m * x_sum) / node_count;
        return 0;
    }

    /**
     * Restores the nodes to the state before a trial started. It attempts to take the popped 
     * nodes and re-sort them into the array.
     */
    void Ransac::restore_trial(node_t nodes[], int ref_index, int original_size, int & size) {
        // sort the popped nodes in the void region ("size" to "original_trial_size")
        //if (debug_enabled) print_nodes(nodes, size, original_size);
        for (int i = original_size - 1; i > size; i--) {
            // find largest angled node
            int largest = size;
            for (int j = size + 1; j <= i; j++) {
                if (nodes[j].angle > nodes[largest].angle) {
                    largest = j;
                }
            }
            // swap with high node
            node_t high = nodes[i];
            nodes[i] = nodes[largest];
            nodes[largest] = high;
        }
        // copy void nodes to temp array
        //if (debug_enabled) print_nodes(nodes, size, original_size);
        int popped_nodes = original_size - size;
        for (int i = size; i < original_size; i++) {
            temp_nodes[i - size] = nodes[i]; 
        }
        // shift nodes in array to make room for popped void nodes
        for (int i = size - 1; i >= ref_index; i--) {
            nodes[i + popped_nodes] = nodes[i];
            // insert popped nodes
            int temp_index = i - ref_index;
            if (temp_index < popped_nodes) {
                nodes[i] = temp_nodes[temp_index];
            }
        }
        //if (debug_enabled) print_nodes(nodes, size, original_size);
        // restore size
        size = original_size;
    }

    /*
     * Pops the given node by moving it to the end of the array and shifting the rest to the left. The
     * size is then decremented to hide the popped value.
     */
    void Ransac::pop_node(int node, node_t nodes[], int & size) {
        // preserve node to pop
        node_t pop_node = nodes[node];
        // shift all nodes to the left to fill in space
        for (int i = node; i < size - 1; i++) {
            nodes[i] = nodes[i + 1];
        }
        // add popped node to end
        nodes[size - 1] = pop_node;
        // decrement array size
        size--;
    }

    /*
     * Computes the least squared distance from a point to a line.
     */
    float Ransac::dst2_to_line(line_t & line, float x, float y) {
        float numerator = fabs(x - line.m * y - line.b);
        return (numerator * numerator) / (1 + line.m * line.m);
    }
        
}
