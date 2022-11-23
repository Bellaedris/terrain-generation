#include "vec.h"
#include "utils.h"
#include "mesh.h"
#include "FastNoiseLite.h"

#include <vector>

struct areaCell {
    int x;
    int y;
    double height;

    areaCell() : x(0), y(0), height(0) {}
    areaCell(int x, int y, double height) : x(x), y(y), height(height) {}
};

inline bool compareCells(areaCell lh, areaCell rh) {return lh.height > rh.height;}

class Ray;

class ScalarField {
protected:
    vec2 a, b; //box2, terrain bounding box
    int nx, ny; //precision 
    std::vector<double> hm;
    double stepX, stepY;

public:
    ScalarField(vec2 a, vec2 b, int nx, int ny);
    ScalarField(vec2 a, vec2 b, int nx, int ny, const std::vector<double> &hm) : a(a), b(b), nx(nx), ny(ny), hm(hm) {}
    ScalarField& operator+=(const ScalarField &rhs);
    ScalarField& operator+=(const double &rhs);
    ScalarField& operator*=(const ScalarField &rhs);
    ScalarField& operator*=(const double &rhs);
    friend ScalarField operator*(ScalarField lhs, const ScalarField &rhs) {lhs+=rhs;return lhs;};
    friend ScalarField operator*(ScalarField lhs, const double &rhs) {lhs+=rhs;return lhs;};
    friend ScalarField operator+(ScalarField lhs, const ScalarField &rhs) {lhs*=rhs;return lhs;};
    friend ScalarField operator+(ScalarField lhs, const double &rhs) {lhs*=rhs;return lhs;};

    ScalarField& sqrt();
    ScalarField& pow(double f);
    vec2 Gradient(int, int) const;
    int Index(int, int) const;  // index dans le tableau
    double Height(int, int) const; //hauteur en un point

    bool Intersect(const Ray &r, float &t, float k/*constante de lipschitz*/) const;

    void ExportImg(char* filename, float min = 0, float max = 0);
};

class Terrain : public ScalarField {
protected:
    adjacency_list_t CreateAdjacencyList();
    double Cost(int source, int dest);
public:
    Terrain(vec2 a, vec2 b, int nx, int ny, int seed = 1337);
    Terrain(vec2 a, vec2 b, int nx, int ny, std::vector<double> hm) : ScalarField(a, b, nx, ny, hm) {}

    Point Position(int, int) const; //vecteur x y z avec hauteur pour des coord du tableau
    Point Position(double, double) const; // pareil mais avec des coord entieres
    Vector Normal(int, int) const; // calcule la normal au sommet
    void FaceNormal(Vector &normal, const Point &a, const Point &b, const Point &c);
    float Slope(int, int) const;
    double Laplacian(int, int) const;

    // Terrain edition
    void TectonicErosion();
    void CreatePath(int begin, int end);

    float MaxSlope(); // Lipschitzian constant

    bool Inside(Vector) const; // point in the terrain check

    Mesh GenerateMesh();
    ScalarField GetSlope();
    ScalarField GetLaplacian();
    ScalarField GetDrainArea();
};