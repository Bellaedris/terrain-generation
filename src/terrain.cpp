// TODO find interest path to cross
// TODO color of water depends on amount of water (useful for crossings)

#include "terrain.h"
#include "image_io.h"
#include <algorithm>

#pragma region scalar field
ScalarField::ScalarField(vec2 a, vec2 b, int nx, int ny) : a(a), b(b), nx(nx), ny(ny)
{
    stepX = (Point(b) - Point(a)).x / ((double)nx - 1.0);
    stepY = (Point(b) - Point(a)).y / ((double)ny - 1.0);
}

ScalarField &ScalarField::operator+=(const ScalarField &rhs)
{
    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            hm[x * nx + y] = Height(x, y) + rhs.Height(x, y);
        }
    }

    return *this;
}

ScalarField &ScalarField::operator+=(const double &rhs)
{
    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            hm[x * nx + y] = Height(x, y) + rhs;
        }
    }

    return *this;
}

ScalarField &ScalarField::operator*=(const ScalarField &rhs)
{
    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            hm[x * nx + y] = Height(x, y) * rhs.Height(x, y);
        }
    }

    return *this;
}

ScalarField &ScalarField::operator*=(const double &rhs)
{
    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            hm[x * nx + y] = Height(x, y) * rhs;
        }
    }

    return *this;
}

ScalarField &ScalarField::sqrt()
{
    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            hm[x * nx + y] = std::sqrt(Height(x, y));
        }
    }

    return *this;
}

ScalarField &ScalarField::pow(double f)
{
    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            hm[x * nx + y] = std::pow(Height(x, y), f);
        }
    }

    return *this;
}

int ScalarField::Index(int i, int j) const
{
    return i * nx + j;
}

std::pair<int, int> ScalarField::ReverseIndex(int index) const
{
    return {index % nx, index / nx};
}

double ScalarField::Height(int i, int j) const
{
    return hm[Index(i, j)];
}

vec2 ScalarField::Gradient(int i, int j) const
{
    double dhx, dhy;
    if (i == 0)
        dhx = (Height(i + 1, j) - Height(i, j)) / stepX;
    else if (i == nx - 1)
        dhx = (Height(i, j) - Height(i - 1, j)) / stepX;
    else
        dhx = (Height(i + 1, j) - Height(i - 1, j)) / stepX;

    if (j == 0)
        dhy = (Height(i, j + 1) - Height(i, j)) / stepY;
    else if (j == ny - 1)
        dhy = (Height(i, j) - Height(i, j - 1)) / stepY;
    else
        dhy = (Height(i, j + 1) - Height(i, j - 1)) / stepY;

    return vec2(dhx, dhy);
}

void ScalarField::ExportImg(char *filename, float min, float max)
{
    Image out(nx, ny);

    if (min == max)
    {
        // find max and min val
        max = 0;
        min = MAXFLOAT;
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
            float grey = Lerp(0, 1, Normalize01(min, max, Height(x, y)));
            out(x, y) = Color(grey, grey, grey, 1);
        }
    }

    write_image(out, filename);
}

std::pair<double, double> ScalarField::GetMinMax(const ScalarField &sf)
{
    int x, y, i;
    double h;
    double max = 0.;
    double min = MAXFLOAT;
    // find max and min val
    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            h = sf.Height(x, y);
            if (max < h)
                max = h;
            if (min > h)
                min = h;
        }
    }

    return {min, max};
}

#pragma endregion

#pragma region construct / basics
Terrain::Terrain(vec2 a, vec2 b, int nx, int ny, int numberOfCities, int seed) : ScalarField(a, b, nx, ny)
{
    hm = std::vector<double>(nx * ny);
    texture = Image(nx, ny);

    // generates noise
    FastNoiseLite noisegen(seed);
    noisegen.SetNoiseType(FastNoiseLite::NoiseType::NoiseType_OpenSimplex2);
    noisegen.SetFractalType(FastNoiseLite::FractalType::FractalType_FBm);
    noisegen.SetFractalOctaves(4);
    noisegen.SetFractalLacunarity(2);
    noisegen.SetFractalGain(0.4);
    noisegen.SetFrequency(1.28 / (double)nx);

    for (float x = 0; x < nx; x++)
        for (float y = 0; y < ny; y++)
        {
            hm[Index(x, y)] = noisegen.GetNoise(x, y);
        }

    FindInterestPoints(numberOfCities);
}

Terrain::Terrain(vec2 a, vec2 b, int nx, int ny, double heightMax) : ScalarField(a, b, nx, ny)
{
    hm = std::vector<double>(nx * ny);
    texture = Image(nx, ny);

    for (float x = 0; x < nx; x++)
        for (float y = 0; y < ny; y++)
        {
            hm[Index(x, y)] = Lerp(-1, heightMax, y / (float)ny);
        }
}

Point Terrain::Position(int i, int j) const
{
    double z = Height(i, j);
    double x = Lerp(a.x, b.x, i) / (nx - 1); // lerp
    double y = Lerp(a.y, b.y, j) / (ny - 1);

    return Point(x, y, z);
}

double Terrain::Slope(int i, int j) const
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
    neighborhood n = Get4Neighbors(i, j);

    double sum = 0;

    for (int i = 0; i < n.n; i++)
    {
        sum += n.neighbors[i].h;
    }

    sum -= 4 * Height(i, j);

    return sum / (stepX * stepY);
}

neighborhood Terrain::Get4Neighbors(int i, int j) const
{
    neighborhood n;
    n.neighbors.resize(8);

    for (size_t k = 0; k < 4; k++)
    {
        int neighI = i + neigh8x[neigh4Indices[k]];
        int neighJ = j + neigh8y[neigh4Indices[k]];
        int index = neighI * nx + neighJ;

        if (index > 0 && index < (nx * ny - 1))
        {
            n.neighbors[n.n] = grid_neighbor(neighI, neighJ, Slope(neighI, neighJ), Height(neighI, neighJ));
            n.n++;
        }
    }

    return n;
}

neighborhood Terrain::Get8Neighbors(int i, int j) const
{
    neighborhood n;
    n.neighbors.resize(8);
    for (int k = 0; k < 8; k++)
    {
        int neighI = i + neigh8x[k];
        int neighJ = j + neigh8y[k];
        int index = neighI * nx + neighJ;

        if (index > 0 && index < (nx * ny - 1))
        {
            n.neighbors[n.n] = grid_neighbor(neighI, neighJ, Slope(neighI, neighJ), Height(neighI, neighJ));
            n.n++;
        }
    }

    return n;
}

neighborhood Terrain::GetNNeighbors(int n, int i, int j) const
{
    neighborhood ne;
    ne.neighbors.resize((2 + n) * (2 + n));

    int index, neighI, neighJ;
    for (int x = -n; x <= n; x++)
        for (int y = -n; y <= n; y++)
        {
            index = Index(i + x, j + y);
            neighI = i + x;
            neighJ = j + y;
            if (index > 0 && index < (nx * ny - 1) && !(x == 0 && y == 0))
            {
                ne.neighbors[ne.n] = grid_neighbor(neighI, neighJ, Slope(neighI, neighJ), Height(neighI, neighJ));
                ne.n++;
            }
        }
    return ne;
}

#pragma endregion

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
            m.texcoord(x / (double)nx, y / (double)ny);
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

void Terrain::GenerateTexture()
{
    ScalarField a = GetDrainArea();
    double max = a.Height(0, 0);
    double min = a.Height(0, 0);
    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            double h = a.Height(x, y);
            if (max < h)
                max = h;
            if (min > h)
                min = h;
        }
    }

    ScalarField s = GetSlope();
    auto minMax = GetMinMax(s);

    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            if (a.Height(x, y) > minWater * max)
            {
                double normalized = Normalize01(minWater * max, 0.05 * max, a.Height(x, y));
                texture(x, y) = Lerp(Color(0, .39, .61), Color(0.26, 0.69, 0.8), normalized);
                continue;
            }
            /*if (Height(x, y) < -0.5)
                texture(x, y) = Color(0.73, 0.5, 0.37);
            else if (Height(x, y) < 0)
                texture(x, y) = Color(0.13, 0.54, 0.13);
            else if (Height(x, y) < 0.5)
                texture(x, y) = Color(0.5, 0.5, 0.5);
            else
                texture(x, y) = White();*/
            if (Height(x, y) < 0.2)
                texture(x, y) = Lerp(Color(0.13, 0.54, 0.13), Color(0.73, 0.5, 0.37), Normalize01(minMax.first, minMax.second, s.Height(x, y)));
            else if (Height(x, y) < 0.3)
                if (s.Height(x, y) < 0.5)
                    texture(x, y) = White();
                else
                    texture(x, y) = Color(0.5, 0.5, 0.5);
            else if (s.Height(x, y) < 0.8)
                texture(x, y) = White();
            else
                texture(x, y) = Color(0.5, 0.5, 0.5);
            // texture(x, y) = Lerp(Color(0.5, 0.5, 0.5), White(), Normalize01(minMax.first, minMax.second, s.Height(x, y)));
        }
    }
}

ScalarField Terrain::GetSlope()
{
    std::vector<double> slopes;
    slopes.resize(hm.size());

    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
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

    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            laplacians[Index(x, y)] = Laplacian(x, y);
        }
    }

    ScalarField sm(a, b, nx, ny, laplacians);
    return sm;
}

ScalarField Terrain::GetDrainArea()
{
    std::vector<double> areas;
    areas.resize(hm.size());

    std::vector<areaCell> sortedCells;
    sortedCells.resize(hm.size());
    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            areas[Index(x, y)] = 1;
            sortedCells[Index(x, y)] = areaCell(x, y, Height(x, y));
        }
    }

    std::sort(sortedCells.begin(), sortedCells.end(), compareCells);

    for (areaCell c : sortedCells)
    {
        // find max slope on neighbors of the current point
        int index = 0;
        double maxHeight = 0.0;
        double curHeight = Height(c.x, c.y);
        double heightDiff;

        neighborhood n = GetNNeighbors(1, c.x, c.y);

        for (int i = 0; i < n.n; i++)
        {
            heightDiff = curHeight - Height(n.neighbors[i].i, n.neighbors[i].j);
            if (heightDiff > maxHeight)
            {
                index = Index(n.neighbors[i].i, n.neighbors[i].j);
                maxHeight = heightDiff;
            }
        }

        areas[index] += areas[Index(c.x, c.y)];
    }

    return ScalarField(a, b, nx, ny, areas);
}

ScalarField Terrain::GetWetness()
{
    ScalarField s = GetSlope();
    ScalarField d = GetDrainArea();

    std::vector<double> wetnesses;
    wetnesses.resize(hm.size());

    double epsilon = 0.0001;

    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            wetnesses[Index(x, y)] = std::log(d.Height(x, y) / (s.Height(x, y) + epsilon));
        }
    }

    ScalarField sm(a, b, nx, ny, wetnesses);
    return sm;
}

void Terrain::TectonicErosion()
{
    double n = 1;
    double hn = n / 2.0;

    for (int i = 0; i < 100; i++)
    {
        ScalarField totalSlope = GetSlope();

        ScalarField totalDrain = GetDrainArea().pow(hn);
        totalDrain *= -0.001;
        totalDrain *= totalSlope;

        ScalarField totalLaplacian = GetLaplacian().pow(n);
        totalLaplacian *= 0.001;
        totalLaplacian += totalDrain;

        *this += totalLaplacian;
        *this += 0.001;
    }
}