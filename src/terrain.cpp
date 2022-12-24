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
    double dhx = (Height(i + 1, j) - Height(i - 1, j)) / stepX;
    double dhy = (Height(i, j + 1) - Height(i, j - 1)) / stepY;

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
            float grey = Lerp(0, 1, Normalize01(min, max, Height(x, y)));
            out(x, y) = Color(grey, grey, grey, 1);
        }
    }

    write_image(out, filename);
}

#pragma endregion

#pragma region construct / basics
Terrain::Terrain(vec2 a, vec2 b, int nx, int ny, int seed) : ScalarField(a, b, nx, ny)
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

    for (int x = 0; x < nx; x++)
    {
        for (int y = 0; y < ny; y++)
        {
            if (a.Height(x, y) > 0.003 * max)
            {
                double normalized = Normalize01(0.003 * max, 0.05 * max, a.Height(x, y));
                texture(x, y) = Lerp(Color(0, .39, .61), Color(0.26, 0.69, 0.8), normalized);
                continue;
            }
            if (Height(x, y) < -0.5)
                texture(x, y) = Color(188 / 255.0, 129 / 255.0, 95 / 255.0);
            else if (Height(x, y) < 0)
                texture(x, y) = Color(34 / 255.0, 139 / 255.0, 34 / 255.0);
            else if (Height(x, y) < 0.5)
                texture(x, y) = Color(0.5, 0.5, 0.5);
            else
                texture(x, y) = White();
        }
    }
}

ScalarField Terrain::GetSlope()
{
    std::vector<double> slopes;
    slopes.resize(hm.size());

    for (int x = 1; x < nx - 1; x++)
    {
        for (int y = 1; y < ny - 1; y++)
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

double Terrain::Cost(int si, int sj, int di, int dj, const ScalarField &wetness)
{
    double d = distance(Position(si, sj), Position(di, dj));
    double slope = std::abs(Slope(si, sj) - Slope(di, dj));

    return d * (1 + 10 * slope + 0.5 * wetness.Height(di, dj));
}

double Terrain::RiverCost(int si, int sj, int di, int dj, const ScalarField &area)
{
    double d = std::abs(area.Height(si, sj) - area.Height(di, dj));

    return d;
}

adjacency_list_t Terrain::CreateAdjacencyList()
{
    // int n = 2; // degree of neighbors
    // int hn = std::floor(n / 2.0);
    adjacency_list_t adj_list(nx * ny);
    ScalarField wetness = GetWetness();

    int newX, newY;

    for (int y = 0; y < ny; y++)
    {
        for (int x = 0; x < nx; x++)
        {
            int i = Index(x, y);
            neighborhood n = GetNNeighbors(2, x, y);
            for (int k = 0; k < n.n; k++)
            {
                newX = n.neighbors[k].i;
                newY = n.neighbors[k].j;
                adj_list[i].push_back(neighbor(Index(newX, newY), Cost(x, y, newX, newY, wetness)));
            }
        }
    }

    return adj_list;
}

adjacency_list_t Terrain::CreateRiverAdjacencyList()
{
    // int n = 2; // degree of neighbors
    // int hn = std::floor(n / 2.0);
    adjacency_list_t adj_list(nx * ny);
    ScalarField drain = GetDrainArea();

    int newX, newY;

    for (int y = 0; y < ny; y++)
    {
        for (int x = 0; x < nx; x++)
        {
            int i = Index(x, y);
            neighborhood n = Get8Neighbors(x, y);
            for (int k = 0; k < n.n; k++)
            {
                newX = n.neighbors[k].i;
                newY = n.neighbors[k].j;
                adj_list[i].push_back(neighbor(Index(newX, newY), RiverCost(x, y, newX, newY, drain)));
            }
        }
    }

    return adj_list;
}

void Terrain::CreatePath(int begin, int end)
{
    std::vector<weight_t> minDistance;
    std::vector<vertex_t> previous;
    DijkstraComputePaths(begin, CreateAdjacencyList(), minDistance, previous);

    std::list<vertex_t> path = DijkstraGetShortestPathTo(end, previous);

    for (auto it = path.begin(); it != path.end(); it++)
    {
        std::pair<int, int> indices = ReverseIndex(*it);
        texture(indices.first, indices.second) = Color(.52, .41, .44);
    }
}

void Terrain::CreateRiver()
{
    ScalarField a = GetDrainArea();
    int begin, end;

    // std::vector<double> ahm(nx * ny);
    // for (int x = 1; x < nx - 1; x++)
    // {
    //     for (int y = 1; y < ny - 1; y++)
    //     {
    //         ahm[x * nx + y] = a.Height(x, y);
    //     }
    // }

    // std::sort(ahm.begin(), ahm.end(), sortGreaterThan);

    // srand(time(NULL));
    // int randIndex = rand() % 10;
    // begin = ahm[randIndex];

    // std::sort(ahm.begin(), ahm.end(), sortLowerThan);

    // randIndex = rand() % 10;
    // end = ahm[randIndex];

    // std::cout << begin << " " << end << std::endl;

    double max = 0;
    double min = MAXFLOAT;
    for (int x = 1; x < nx - 1; x++)
    {
        for (int y = 1; y < ny - 1; y++)
        {
            double h = a.Height(x, y);
            if (max < h)
            {
                max = h;
                end = Index(x, y);
            }
            if (min > h)
            {
                min = h;
                begin = Index(x, y);
            }
        }
    }

    std::vector<weight_t> minDistance;
    std::vector<vertex_t> previous;
    DijkstraComputePaths(begin, CreateRiverAdjacencyList(), minDistance, previous);

    std::list<vertex_t> path = DijkstraGetShortestPathTo(end, previous);

    for (auto it = path.begin(); it != path.end(); it++)
    {
        std::cout << *it << std::endl;
        std::pair<int, int> indices = ReverseIndex(*it);
        texture(indices.first, indices.second) = Red(); // Color(0, .39, .61);
    }
}