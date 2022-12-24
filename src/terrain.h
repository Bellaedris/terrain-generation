#include "vec.h"
#include "utils.h"
#include "mesh.h"
#include "FastNoiseLite.h"
#include "image.h"

#include <vector>

struct grid_neighbor {
    int i, j; // coord
    double s; // slope
    double h; // height

    grid_neighbor(): i(-1), j(-1), s(-1), h(-1) {}
    grid_neighbor(int i, int j, double s, double h) : i(i), j(j), s(s), h(h) {}
};

struct neighborhood {
    std::vector<grid_neighbor> neighbors; // list of neighbors
    int n; // number of neighbors that are really usable

    neighborhood() : n(0) {}
};

struct areaCell {
    int x;
    int y;
    double height;

    areaCell() : x(0), y(0), height(0) {}
    areaCell(int x, int y, double height) : x(x), y(y), height(height) {}
};

inline bool compareCells(areaCell lh, areaCell rh) {return lh.height > rh.height;}
inline bool sortGreaterThan(double i, double j) {return i > j;};
inline bool sortLowerThan(double i, double j) {return i < j;};

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
    std::pair<int, int> ReverseIndex(int) const; // index dans le tableau to 2 coord
    double Height(int, int) const; //hauteur en un point

    bool Intersect(const Ray &r, float &t, float k/*constante de lipschitz*/) const;

    void ExportImg(char* filename, float min = 0, float max = 0);
};

class Terrain : public ScalarField {
protected:
    adjacency_list_t CreateAdjacencyList();
    adjacency_list_t CreateRiverAdjacencyList();
    double Cost(int si, int sj, int di, int dj, const ScalarField &wetness);
    double RiverCost(int si, int sj, int di, int dj, const ScalarField &wetness);

    neighborhood Get8Neighbors(int i, int j) const;
    neighborhood Get4Neighbors(int i, int j) const;
    neighborhood GetNNeighbors(int n, int i, int j) const;

    int neigh4Indices[4] = {1, 6, 4, 3};

    const int neigh8x[8] = {
        -1, 0, 1,
        -1,    1,
        -1, 0, 1
    };

    const int neigh8y[8] = {
        -1, -1, -1,
         0,      0,
         1,  1,  1
    };

    Image texture;

public:
    Terrain(vec2 a, vec2 b, int nx, int ny, int seed = 1337) ;
    Terrain(vec2 a, vec2 b, int nx, int ny, std::vector<double> hm) : ScalarField(a, b, nx, ny, hm) {}
    Terrain(vec2 a, vec2 b, int nx, int ny, double heightMax);

    Image& GetTexture() {return texture;};

    Point Position(int, int) const; //vecteur x y z avec hauteur pour des coord du tableau
    Point Position(double, double) const; // pareil mais avec des coord entieres
    Vector Normal(int, int) const; // calcule la normal au sommet
    void FaceNormal(Vector &normal, const Point &a, const Point &b, const Point &c);
    double Slope(int, int) const;
    double Laplacian(int, int) const;

    // Terrain edition
    void TectonicErosion();
    void CreatePath(int begin, int end);
    void CreateRiver();

    float MaxSlope(); // Lipschitzian constant

    bool Inside(Vector) const; // point in the terrain check

    Mesh GenerateMesh();
    void GenerateTexture();
    ScalarField GetSlope();
    ScalarField GetLaplacian();
    ScalarField GetDrainArea();
    ScalarField GetWetness();
};