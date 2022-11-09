#include "vec.h"
#include "utils.h"
#include "mesh.h"
#include "FastNoiseLite.h"

#include <vector>

class Terrain {
protected:
    vec2 a, b; //box2, terrain bounding box
    int nx, ny; //precision 
    std::vector<double> hm;

public:
    Terrain(vec2 a, vec2 b, int nx, int ny);

    int Index(int, int) const;  // index dans le tableau
    double Height(int, int) const; //hauteur en un point
    Point Position(int, int) const; //vecteur x y z avec hauteur pour des coord du tableau
    Point Position(double, double) const; // pareil mais avec des coord entieres
    Vector Normal(int, int) const; // calcule la normal au sommet
    vec2 Gradient(int, int) const;
    float Slope(int, int) const;

    bool Inside(Vector) const; // point in the terrain check

    Mesh GenerateMesh();
};