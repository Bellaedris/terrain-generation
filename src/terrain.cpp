#include "terrain.h"
#include "image.h"
#include "image_io.h"

ScalarField::ScalarField(vec2 a, vec2 b, int nx, int ny) : a(a), b(b), nx(nx), ny(ny)
{
    stepX = (Point(b) - Point(a)).x / (nx - 1);
    stepY = (Point(b) - Point(a)).y / (ny - 1);
}

int ScalarField::Index(int i, int j) const
{
    return i * nx + j;
}

double ScalarField::Height(int i, int j) const
{
    return hm[Index(i, j)];
}

vec2 ScalarField::Gradient(int i, int j) const
{
    double dhx = (Height(i + 1, j) - Height(i - 1, j)) / (2 * ((b.x - a.x) / nx - 1));
    double dhy = (Height(i + 1, j) - Height(i - 1, j)) / (2 * ((b.y - a.y) / ny - 1));
    return vec2(dhx, dhy);
}

void ScalarField::ExportImg(char *filename, float min, float max)
{
    Image out(nx, ny);

    if (min == max)
    {
        // find max and min val
        max = Height(0, 0);
        min = Height(0, 0);
        for (int x = 0; x < nx; x++)
        {
            for (int y = 0; y < ny; y++)
            {
                double h = Height(x, y);
                if (max < h)
                    max = h;
                if (min > h)
                    min = h;
            }
        }
    }

    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            float grey = Lerp(0, 1, (Height(x, y) - min) / (max - min));
            out(x, y) = Color(grey, grey, grey, 1);
        }
    }

    write_image(out, filename);
}

Terrain::Terrain(vec2 a, vec2 b, int nx, int ny) : ScalarField(a, b, nx, ny)
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

Point Terrain::Position(int i, int j) const
{
    double z = Height(i, j);
    double x = Lerp(a.x, b.x, i) / (nx - 1); // lerp
    double y = Lerp(a.y, b.y, j) / (ny - 1);

    return Point(x, y, z);
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

double Terrain::Laplacian(int i, int j) const
{
    double xn = Height(i + 1, j);
    double xp = Height(i - 1, j);
    double yn = Height(i, j + 1);
    double yp = Height(i, j - 1);

    return (xn + xp + yn + yp - 4 * Height(i, j)) / (stepX * stepY);
}

/**
 * computes the normal of a face
 */
void Terrain::FaceNormal(Vector &normal, const Point &a, const Point &b, const Point &c)
{
    Vector AB, AC;

    AB = b - a;

    AC = c - a;

    normal = cross(AB, AC);
}

Mesh Terrain::GenerateMesh()
{
    Mesh m(GL_TRIANGLES);

    // write vertices
    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            m.vertex(Position(x, y));
        }
    }

    // write triangles
    for (int x = 0; x < nx - 1; x++)
    {
        for (int y = 0; y < ny - 1; y++)
        {
            m.triangle(x * nx + y, (x + 1) * nx + y, (x + 1) * nx + y + 1);
            m.triangle(x * nx + y, (x + 1) * nx + y + 1, x * nx + y + 1);
        }
    }

    // smoothes the normals
    Vector curNormal;
    std::vector<Vector> normals;
    normals.resize(hm.size());
    for (size_t i = 0; i < m.indices().size(); i += 3)
    {
        int ia = m.indices()[i];
        int ib = m.indices()[i + 1];
        int ic = m.indices()[i + 2];

        Point a = m.positions()[ia];
        Point b = m.positions()[ib];
        Point c = m.positions()[ic];

        FaceNormal(curNormal, a, b, c);

        normals[ia] = normals[ia] + curNormal;
        normals[ib] = normals[ib] + curNormal;
        normals[ic] = normals[ic] + curNormal;
    }

    // normalize all normals
    for (size_t i = 0; i < normals.size(); i++)
    {
        m.normal(normalize(normals[i]));
    }

    return m;
}

ScalarField Terrain::GetSlope()
{
    std::vector<double> slopes;
    slopes.resize(hm.size());
    for (int x = 0; x < nx - 1; x++)
    {
        for (int y = 0; y < ny - 1; y++)
        {
            slopes[Index(x, y)] = Slope(x, y);
        }
    }

    ScalarField sm(a, b, nx, ny, slopes);
    return sm;
}

ScalarField Terrain::GetLaplacian()
{
    std::vector<double> laplacians;
    laplacians.resize(hm.size());
    for (int x = 0; x < nx - 1; x++)
    {
        for (int y = 0; y < ny - 1; y++)
        {
            laplacians[Index(x, y)] = Laplacian(x, y);
        }
    }

    ScalarField sm(a, b, nx, ny, laplacians);
    return sm;
}

ScalarField Terrain::GetDrainArea()
{
    std::vector<double> areas(nx * ny);
    std::vector<areaCell> sortedCells(nx * ny);
    for (int x = 0; x < nx - 1; x++)
    {
        for (int y = 0; y < ny - 1; y++)
        {
            areas[x * nx + y] = 1;
            sortedCells[x * nx + y] = areaCell(x * nx + y, Height(x, y));
        }
    }

    std::sort(sortedCells.begin(), sortedCells.end(), compareCells);

    for (areaCell c : sortedCells)
    {
        
    }
}