#include "ransac.h"

namespace rplidar { namespace algorithms {
	
	Ransac::Ransac(int max_trials, int sample_size, float sample_deviation, float proximity_epsilon, int line_consensus) :
		max_trials(max_trials), sample_size(sample_size),
		sample_deviation(sample_deviation), proximity_epsilon(proximity_epsilon),
		line_consensus(line_consensus) {
		// allocate temp array
		temp_nodes = new node_t[sample_size];
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
		int original_size = size; // size at the beginning of computation
		int trial_count = 0;
		while (size > 0 && trial_count < max_trials) {
			int original_trial_size = size; // size at the beginning of each trial
			// choose a random reference node
			int ref_index = rand() % size;
			float ref_angle = get_angle(nodes[ref_index]);
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
				// validate that pick is within given deviation
				if (fabs(get_angle(nodes[pick]) - ref_angle) <= sample_deviation) {
					// pick the node
					pop_node(pick, nodes, size);
					// correct reference node position when popping from the left
					if (pick < ref_index) ref_index--;
				}
			}
			// pop the reference node
			pop_node(ref_index, nodes, size);
			// compute least squares regression line
			int sample_start = size;              // inclusive
			int sample_end = original_trial_size; // exclusive
			line_t line;
			compute_reg_line(sample_start, sample_end, nodes, line);
			// associate other nodes with the line (by popping them)
			for (int i = 0; i < size; i++) {
				vec2_t vec;
				get_cartesian(nodes[i], vec);
				float dst2 = dst2_to_line(line, vec.x, vec.y);
				if (dst2 <= proximity_epsilon * proximity_epsilon) {
					// node is close enough to line
					pop_node(i, nodes, size);
					i--;	
				}
			}
			// confirm line by given consensus count
			int popped_nodes = original_trial_size - size;
			if (popped_nodes >= line_consensus) {
				// recompute the regression line with all associated nodes
				compute_reg_line(size, original_trial_size, nodes, line);
				// push into line array
				reg_lines.push_back(line);
			} else {
				// put nodes back into trial array
				restore_trial(nodes, ref_index, original_trial_size, size);
			}
			// increment the trial counter
			trial_count++;
		}
	}


	line_t Ransac::compute_reg_line(int start, int end, node_t nodes[], line_t & line) {
		vec2_t vec;
		int node_count = start - end;
		// compute sums for regression line
		float x_sum = 0;
		float y_sum = 0;
		float x2_sum = 0;
		float xy_sum = 0; 
		for (int i = start; i < end; i++) {
			get_cartesian(nodes[i], vec);
			x_sum += vec.x;
			y_sum += vec.y;
			x2_sum += vec.x * vec.x;
			xy_sum += vec.x * vec.y;
		}
		// compute slope
		line.m = (node_count * xy_sum - x_sum * y_sum) / 
			 (node_count * x2_sum - x_sum * x_sum);
		// compute y-intercept
		line.b = (y_sum - line.m * x_sum) / node_count;
		return line;
	}

	/**
	 * Restores the nodes to the state before a trial started. It attempts to take the popped 
	 * nodes and re-sort them into the array.
	 */
	void Ransac::restore_trial(node_t nodes[], int ref_index, int original_trial_size, int & size) {
		// sort the popped nodes in the void region ("size" to "original_trial_size")
		for (int i = original_trial_size - 1; i > size; i--) {
			// find largest angled node
			int largest = size;
			for (int j = size + 1; j <= i; j++) {
				if (get_angle(nodes[j]) > get_angle(nodes[largest])) {
					largest = j;
				}
			}
			// swap with high node
			node_t high = nodes[i];
			nodes[i] = nodes[largest];
			nodes[largest] = high;
		}
		// copy void nodes to temp array
		for (int i = size; i < original_trial_size; i++) {
			temp_nodes[i - size] = nodes[i]; 
		}
		// shift nodes in array to make room for popped void nodes
		int popped_nodes = original_trial_size - size;
		for (int i = ref_index; i < size; i++) {
			nodes[i + popped_nodes] = nodes[i];
			// insert popped nodes
			int temp_index = i - ref_index;
			if (temp_index < popped_nodes) {
				nodes[i] = temp_nodes[temp_index];
			}
		}
		// restore size
		size = original_trial_size;
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
		
}}
