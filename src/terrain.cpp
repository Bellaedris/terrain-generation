#include "terrain.h"

Terrain::Terrain(vec2 a, vec2 b, int nx, int ny) : a(a), b(b), nx(nx), ny(ny)
{
    hm = std::vector<double>(nx * ny);

    // generates noise
    FastNoiseLite noisegen(time(NULL));
    noisegen.SetNoiseType(FastNoiseLite::NoiseType::NoiseType_OpenSimplex2);
    noisegen.SetFractalType(FastNoiseLite::FractalType::FractalType_FBm);
    noisegen.SetFractalOctaves(4);
    noisegen.SetFractalLacunarity(2);
    noisegen.SetFractalGain(0.4);
    noisegen.SetFrequency(0.01);

    for (float x = 0; x < nx; x++)
        for (float y = 0; y < ny; y++)
        {
            hm[Index(x, y)] = noisegen.GetNoise(x, y);
        }
}

int Terrain::Index(int i, int j) const
{
    return i * nx + j;
}

double Terrain::Height(int i, int j) const
{
    return hm[Index(i, j)];
}

Point Terrain::Position(int i, int j) const
{
    double z = Height(i, j);
    double x = a.x + ((b.x - a.x) * i) / (nx - 1); // lerp 
    double y = a.y + ((b.y - a.y) * j) / (ny - 1);

    return Point(x, y, z);
}

vec2 Terrain::Gradient(int i, int j) const
{
    double dhx = (Height(i + 1, j) - Height(i - 1, j)) / (2 * ((b.x - a.x) / nx - 1));
    double dhy = (Height(i + 1, j) - Height(i - 1, j)) / (2 * ((b.y - a.y) / ny - 1));
    return vec2(dhx, dhy);
}

float Terrain::Slope(int i, int j) const
{
    return vec2length(Gradient(i, j));
}

Vector Terrain::Normal(int i, int j) const
{
    vec2 grad = Gradient(i, j);
    Vector norm(-grad.x, -grad.y, 1);
    return normalize(norm);
}

Mesh Terrain::GenerateMesh()
{
    Mesh m(GL_TRIANGLES);
    
    for(int x = 0; x < nx; x++) {
        for(int y = 0; y < ny; y++) {
            m.vertex(Position(x, y));
            m.normal(Normal(x, y));
        }
    }

    for(int x = 0; x < nx - 1; x++) {
        for(int y = 0; y < ny - 1; y++) {
            m.triangle(x * nx + y, (x + 1) * nx + y, (x + 1) * nx + y + 1);
            m.triangle(x * nx + y, (x + 1) * nx + y + 1, x * nx + y + 1);
        }
    }

    return m;
}