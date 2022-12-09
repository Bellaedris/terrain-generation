#include "utils.h"

double vec2length2(const vec2 &v) {
    return v.x * v.x + v.y * v.y;
}

double vec2length(const vec2 &v) {
    return std::sqrt(vec2length2(v));
}

double Lerp(double a, double b, double t) {
    return a + ((b - a) * t);
}

Color Lerp(Color a, Color b, double f) {
    return Color(
        Lerp(a.r, b.r, f),
        Lerp(a.g, b.g, f),
        Lerp(a.b, b.b, f)
    );
}


/**
 * @brief Normalize01 recenters a value between 0 and 1
 * @param min the minimal value of the interval
 * @param max the maximal value of the interval
 * @param t the value to interpolate
 * @return t normalized between 0 and 1. Returns 0 if t is min, 1 if t is max
 */
double Normalize01(double min, double max, double t) {
    return (t - min) / (max - min);
}


double Clamp(double min, double max, double v)
{
    if (v < min)
        return min;
    if (v > max)
        return max;
    return v;
}

void DijkstraComputePaths(vertex_t source,
                          const adjacency_list_t &adjacency_list,
                          std::vector<weight_t> &min_distance,
                          std::vector<vertex_t> &previous)
{
    int n = adjacency_list.size();
    min_distance.clear();
    min_distance.resize(n, max_weight);
    min_distance[source] = 0;
    previous.clear();
    previous.resize(n, -1);
    std::set<std::pair<weight_t, vertex_t> > vertex_queue;
    vertex_queue.insert(std::make_pair(min_distance[source], source));

    while (!vertex_queue.empty()) 
    {
        weight_t dist = vertex_queue.begin()->first;
        vertex_t u = vertex_queue.begin()->second;
        vertex_queue.erase(vertex_queue.begin());

        // Visit each edge exiting u
	const std::vector<neighbor> &neighbors = adjacency_list[u];
        for (std::vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
             neighbor_iter != neighbors.end();
             neighbor_iter++)
        {
            vertex_t v = neighbor_iter->target;
            weight_t weight = neighbor_iter->weight;
            weight_t distance_through_u = dist + weight;
	    if (distance_through_u < min_distance[v]) {
	        vertex_queue.erase(std::make_pair(min_distance[v], v));

	        min_distance[v] = distance_through_u;
	        previous[v] = u;
	        vertex_queue.insert(std::make_pair(min_distance[v], v));

	    }

        }
    }
}

std::list<vertex_t> DijkstraGetShortestPathTo(
    vertex_t vertex, const std::vector<vertex_t> &previous)
{
    std::list<vertex_t> path;
    for ( ; vertex != -1; vertex = previous[vertex])
        path.push_front(vertex);
    return path;
}