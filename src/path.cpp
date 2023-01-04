#include "terrain.h"

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

adjacency_list_t Terrain::CreateAdjacencyList(int degree)
{
    // int n = 2; // degree of neighbors
    // int hn = std::floor(n / 2.0);
    adjacency_list_t adj_list(nx * ny);
    ScalarField wetness = GetWetness();

    int i, newX, newY;

    for (int y = 0; y < ny; y++)
    {
        for (int x = 0; x < nx; x++)
        {
            i = Index(x, y);
            neighborhood n = GetNNeighbors(degree, x, y);
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

void Terrain::CreatePath(int begin, int end, int degree)
{
    if (map_adjacency.size() == 0)
        map_adjacency = CreateAdjacencyList(degree);

    std::vector<weight_t> minDistance;
    std::vector<vertex_t> previous;
    DijkstraComputePaths(begin, map_adjacency, minDistance, previous);

    std::list<vertex_t> path = DijkstraGetShortestPathTo(end, previous);

    if (degree < 2)
        for (auto it = path.begin(); it != path.end(); it++)
        {
            std::pair<int, int> indices = ReverseIndex(*it);
            texture(indices.first, indices.second) = Color(.52, .41, .44);
        }
    else
    {
        neighborhood n;
        for (auto it = path.begin(); it != path.end(); it++)
        {
            std::pair<int, int> indices = ReverseIndex(*it);
            texture(indices.first, indices.second) = Color(.52, .41, .44);
            n = GetNNeighbors(degree - 1, indices.first, indices.second);
            for (int i = 0; i < n.n; i++)
                texture(n.neighbors[i].i, n.neighbors[i].j) = Color(.52, .41, .44);
        }
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

int Terrain::GetPathLength(int begin, int end, int degree)
{
    if (map_adjacency.size() == 0)
        map_adjacency = CreateAdjacencyList(degree);

    std::vector<weight_t> minDistance;
    std::vector<vertex_t> previous;
    DijkstraComputePaths(begin, map_adjacency, minDistance, previous);

    return DijkstraGetShortestPathTo(end, previous).size();
}

void Terrain::ConnectCities()
{
    int best, firstSeg, secondSeg;
    cityScore middlePath;
    bool foundBetter;

    for (cityScore origin : cities)
    {
        for (cityScore dest : cities)
        {
            foundBetter = false;
            if (origin.i == dest.i && origin.j == dest.j)
                continue;
            best = GetPathLength(Index(origin.j, origin.i), Index(dest.j, dest.i), 1);
            for (cityScore inBetween : cities)
            {
                if ((inBetween.i == origin.i && inBetween.j == origin.j) ||
                    (inBetween.i == dest.i && inBetween.j == dest.j))
                    continue;
                firstSeg = GetPathLength(Index(origin.j, origin.i), Index(inBetween.j, inBetween.i), 1);
                secondSeg = GetPathLength(Index(inBetween.j, inBetween.i), Index(dest.j, dest.i), 1);

                if (std::pow(firstSeg, 2) + std::pow(secondSeg, 2) <= std::pow(best, 2))
                {
                    foundBetter = true;
                    middlePath = inBetween;
                    break;
                }
            }
            if (foundBetter)
            {
                CreatePath(Index(origin.j, origin.i), Index(middlePath.j, middlePath.i), 1);
                CreatePath(Index(middlePath.j, middlePath.i), Index(dest.j, dest.i), 1);
            }
            else
                CreatePath(Index(origin.j, origin.i), Index(dest.j, dest.i), 1);
        }
    }
}