#include "terrain.h"

std::vector<cityScore> Terrain::FindInterestPoints(int numberOfPoints = 1)
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

    std::vector<cityScore> best(numberOfPoints);
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
                    best[0].UpdateValues(x, y, w);
                    bestW = w;
                }
            else
                for(i = 1; i < numberOfPoints; i++)
                    if (w > best[i].w){
                        best[i].UpdateValues(x, y, w);
                        break;
                    }
        }
    for(i = 0; i < numberOfPoints; i++) {
        texture(best[i].i, best[i].j) = Red();
    }
    return best;
}

