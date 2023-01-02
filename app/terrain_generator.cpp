#include "terrain.h"

#include "wavefront.h"
#include "texture.h"
#include "path.cpp"
#include "cities.cpp"

#include "orbiter.h"
#include "draw.h"        
#include "app_camera.h"        // classe Application a deriver

// utilitaire. creation d'une grille / repere.
Mesh make_grid( const int n= 10 )
{
    Mesh grid= Mesh(GL_LINES);
    
    // grille
    grid.color(White());
    for(int x= 0; x < n; x++)
    {
        float px= float(x) - float(n)/2 + .5f;
        grid.vertex(Point(px, 0, - float(n)/2 + .5f)); 
        grid.vertex(Point(px, 0, float(n)/2 - .5f));
    }

    for(int z= 0; z < n; z++)
    {
        float pz= float(z) - float(n)/2 + .5f;
        grid.vertex(Point(- float(n)/2 + .5f, 0, pz)); 
        grid.vertex(Point(float(n)/2 - .5f, 0, pz)); 
    }
    
    // axes XYZ
    grid.color(Red());
    grid.vertex(Point(0, .1, 0));
    grid.vertex(Point(1, .1, 0));
    
    grid.color(Green());
    grid.vertex(Point(0, .1, 0));
    grid.vertex(Point(0, 1, 0));
    
    grid.color(Blue());
    grid.vertex(Point(0, .1, 0));
    grid.vertex(Point(0, .1, 1));
    
    glLineWidth(2);
    
    return grid;
}

class TP : public AppCamera
{
public:
    // constructeur : donner les dimensions de l'image, et eventuellement la version d'openGL.
    TP( ) : AppCamera(1024, 640) {}
    
    // creation des objets de l'application
    int init( )
    {
       // decrire un repere / grille 
        m_repere= make_grid(10);

        Terrain terrain(vec2(0, 0), vec2(10, 10), 256, 256, 1);

        terrain.ExportImg("../out/heightmap.png", -1, 1);

        ScalarField slope = terrain.GetSlope();
        slope.ExportImg("../out/slopemap.png");

        terrain.GetLaplacian().ExportImg("../out/laplacianmap.png");

        terrain.GetDrainArea().sqrt().sqrt().ExportImg("../out/drainmap.png");

        terrain.GetWetness().ExportImg("../out/wetnessmap.png");

        //terrain.TectonicErosion();
        //terrain.ExportImg("../out/erodedmap.png", -1, 1);

        terrain.GenerateTexture();
        terrain.CreateCitiesAndRoads(5);
        //terrain.CreateRiver();

        m_objet = terrain.GenerateMesh();
        Image i_texture = terrain.GetTexture();
        write_image(i_texture, "../out/texture.png");
        texture = make_texture(0, i_texture); //make_texture(0, i_texture);

        Point pmin, pmax;
        m_objet.bounds(pmin, pmax);
        m_camera.lookat(pmin, pmax);
        m_camera.rotation(180, 0);

        std::cout << "init ok, rendering..." << std::endl;
        
        // etat openGL par defaut
        glClearColor(0.2f, 0.2f, 0.2f, 1.f);        // couleur par defaut de la fenetre
        
        glClearDepth(1.f);                          // profondeur par defaut
        glDepthFunc(GL_LESS);                       // ztest, conserver l'intersection la plus proche de la camera
        glEnable(GL_DEPTH_TEST);                    // activer le ztest

        return 0;   // pas d'erreur, sinon renvoyer -1
    }
    
    // destruction des objets de l'application
    int quit( )
    {
        m_objet.release();
        return 0;   // pas d'erreur
    }
    
    // dessiner une nouvelle image
    int render( )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        draw(m_objet, m_camera, texture);

        return 1;
    }

protected:
    Mesh m_objet;
    Mesh m_repere;

    GLuint texture;
};


int main( int argc, char **argv )
{
    // il ne reste plus qu'a creer un objet application et la lancer 
    TP tp;
    tp.run();
    
    return 0;
}
