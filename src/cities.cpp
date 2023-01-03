#include "terrain.h"

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
    double max = 0.;
    double min = MAXFLOAT;
    for (x = 0; x < nx; x++)
    {
        for (y = 0; y < ny; y++)
        {
            double h = drain.Height(x, y);
            if (max < h)
                max = h;
            if (min > h)
                min = h;
        }
    }

    cities = std::vector<cityScore>(numberOfPoints);
    weight_t bestW;

    double w;
    double h, s, d;
    for (x = 0; x < nx; x++)
        for (y = 0; y < ny; y++)
        {
            // in water, ignore
            if (drain.Height(x, y) > minWater * max)
                continue;
            h = Normalize01(-1, 1, Height(x, y));
            s = 1.0 / (1. + slope.Height(x, y));
            d = drain.Height(x, y);
            w = 1. * h + 10. * s + 0.05 * d;
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
}

void Terrain::GrowAndShowCities(int iter)
{
    const int cityRadius = 1;
    const int spaceBetweenHouses = 2;
    srand(time(NULL));

    int neigh, offsetX, offsetY;
    double angle;
    for (cityScore c : cities)
    {
        std::vector<buildingData *> buildings;
        buildings.push_back(new buildingData(c.i, c.j));

        buildingData *currentBuilding;
        for (int i = 0; i < iter; i++)
        {
            // choose a building that can expand
            do
            {
                int random = rand() % buildings.size();
                std::cout << buildings.size() << " selected " << random << std::endl;
                currentBuilding = buildings[random];
            } while (currentBuilding->canGrow > 8);

            // grow the building block
            do
            {
                neigh = rand() % 8 + 1;
            } while (currentBuilding->neigh[neigh] != nullptr);

            angle = Deg2rad(360.0 / (double) neigh);
            offsetX = std::cos(angle) * (cityRadius + spaceBetweenHouses);
            offsetY = std::sin(angle) * (cityRadius + spaceBetweenHouses);
            fprintf(stderr, "%d %d\n", offsetX, offsetY);
            buildingData *newBuilding = new buildingData(c.i + offsetX, c.j + offsetY);
            newBuilding->updateNeighbor((neigh + 4) % 8, currentBuilding);

            buildings.push_back(newBuilding);
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