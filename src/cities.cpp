#include "terrain.h"

#include <random>

struct buildingData
{
    /**
     * @brief a building can have 8 neighbors on 8 different angles around him
     */
    buildingData *neigh[8] = {nullptr};
    int i;
    int j;
    int canGrow;

    buildingData(int i, int j) : i(i), j(j), canGrow(0) {}
    inline void updateNeighbor(int i, buildingData *b)
    {
        neigh[i] = b;
        canGrow++;
    }
};

void Terrain::FindInterestPoints(int numberOfPoints = 1)
{
    // minimize slope, maximize height, be as close to water as possible
    ScalarField slope = GetSlope();
    ScalarField drain = GetDrainArea();

    int x, y, i;
    auto minMax = GetMinMax(drain);

    cities = std::vector<cityScore>(numberOfPoints);
    weight_t bestW;

    double w;
    double h, s, d;
    std::vector<double> interestPoints(hm.size());
    for (x = 0; x < nx; x++)
        for (y = 0; y < ny; y++)
        {
            // in water, ignore
            if (drain.Height(x, y) > minWater * minMax.second)
                continue;
            h = Normalize01(-1, 1, Height(x, y));
            s = 1.0 / (1. + slope.Height(x, y));
            d = Normalize01(minMax.first, minMax.second, drain.Height(x, y));
            w = 1. * h + 10. * s + 2000 * d;
            interestPoints[Index(x, y)] = w;
            if (w > bestW)
            {
                cities[0].UpdateValues(x, y, w);
                bestW = w;
            }
            else
                for (i = 1; i < numberOfPoints; i++)
                    if (w > cities[i].w)
                    {
                        cities[i].UpdateValues(x, y, w);
                        break;
                    }
        }

    ScalarField ip(a, b, nx, ny, interestPoints);
    ip.ExportImg("../out/interestmap.png");
}

void Terrain::GrowAndShowCities(int iter)
{
    if (mapUpdated)
    {
        FindInterestPoints(cities.size());
        mapUpdated = false;
    }

    const int cityRadius = 0;
    const int spaceBetweenHouses = 1;
    const double step = 360.0 / 8.0;

    ScalarField drain = GetDrainArea();
    double maxDrain = GetMinMax(drain).second;

    std::default_random_engine generator(time(NULL));
    std::uniform_int_distribution<int> randDir(1, 8);
    std::uniform_int_distribution<int> randBuilding;

    int neigh, offsetX, offsetY, i, randomBuilding;
    double angle;
    for (cityScore c : cities)
    {
        std::vector<buildingData *> buildings;
        buildings.push_back(new buildingData(c.i, c.j));

        buildingData *currentBuilding;
        i = 0;
        while (i < iter)
        {
            randBuilding = std::uniform_int_distribution<int>(0, buildings.size() - 1);
            // choose a building that can expand
            do
            {
                randomBuilding = randBuilding(generator);
                currentBuilding = buildings[randomBuilding];
            } while (currentBuilding->canGrow > 8);

            // grow the building block
            do
            {
                neigh = randDir(generator);
                angle = Deg2rad(step * (double)neigh);
                Vector dir(std::cos(angle), std::sin(angle), 0);
                dir = normalize(dir);
                offsetX = dir.x * (cityRadius + spaceBetweenHouses);
                offsetY = dir.y * (cityRadius + spaceBetweenHouses);
            } while (
                currentBuilding->neigh[neigh] != nullptr ||
                drain.Height(currentBuilding->i + offsetX, currentBuilding->j + offsetY) >= minWater * maxDrain);

            buildingData *newBuilding = new buildingData(currentBuilding->i + offsetX, currentBuilding->j + offsetY);
            newBuilding->updateNeighbor((neigh + 4) % 8, currentBuilding);
            currentBuilding->updateNeighbor(neigh, newBuilding);

            buildings.push_back(newBuilding);
            i++;
        }

        int x, y;
        for (size_t i = 0; i < buildings.size(); i++)
        {
            for (x = -cityRadius; x <= cityRadius; x++)
                for (y = -cityRadius; y <= cityRadius; y++)
                    texture(buildings[i]->i + x, buildings[i]->j + y) = Red();
        }
    }
}