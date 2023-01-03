#include "vec.h"
#include "math.h"
#include "color.h"

#include <iostream>
#include <vector>
#include <string>
#include <list>

#include <limits> // for numeric_limits

#include <set>
#include <utility> // for pair
#include <algorithm>
#include <iterator>

double vec2length2(const vec2 &v);

double vec2length(const vec2 &v);

double Lerp(double a, double b, double f);

Color Lerp(Color a, Color b, double f);

double Normalize01(double min, double max, double t);

double Clamp(double min, double max, double val);

inline double Deg2rad(double degree) {return degree * M_PI / 180;};

typedef int vertex_t;
typedef double weight_t;

const weight_t max_weight = std::numeric_limits<double>::infinity();

struct neighbor {
    vertex_t target;
    weight_t weight;
    neighbor(vertex_t arg_target, weight_t arg_weight)
        : target(arg_target), weight(arg_weight) { }
};

typedef std::vector<std::vector<neighbor> > adjacency_list_t;

void DijkstraComputePaths(vertex_t source,
                          const adjacency_list_t &adjacency_list,
                          std::vector<weight_t> &min_distance,
                          std::vector<vertex_t> &previous);


std::list<vertex_t> DijkstraGetShortestPathTo(vertex_t vertex, const std::vector<vertex_t> &previous);