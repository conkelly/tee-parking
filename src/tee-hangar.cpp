#include <algorithm> 
#include <cmath>
#include <vector>

class Node{ 
    public: 
        Node(int x, int y, float cost, int pind): x(x), y(y), cost(cost), pind(pind) {};  
    private: 
        int x; 
        int y; 
        float cost; 
        int pind;
};

class Objects{ 
    public: 
        Objects(std::vector<std::vector<bool>> obmap, int minx, int miny, int maxx, int maxy, int xwidth, int ywidth): obmap(obmap), minx(minx), miny(miny), maxx(maxx), maxy(maxy), xwidth(xwidth), ywidth(ywidth) {}; 
    private: 
        std::vector<std::vector<bool>> obmap; 
        int minx; 
        int miny; 
        int maxx; 
        int maxy; 
        int xwidth; 
        int ywidth;
};

std::vector<float> calc_dist_policy(float gx, float gy, std::vector<float> ox, std::vector<float> oy, float reso, float vr){ 
    Node ngoal(round(gx/reso), round(gy/reso), 0, -1); 
    for (int i = 0; i < ox.size(); i++){ 
        ox[i] = ox[i] / reso;
    }
    for (int i = 0; i < oy.size(); i++){ 
        oy[i] = oy[i] / reso;
    }
    Objects objects = calc_obstacle_map(ox, oy, reso, vr);
};


Node calc_obstacle_map(std::vector<float> ox, std::vector<float> oy, float reso, float vr){ 
    int minx = round( *std::min_element(ox.begin(), ox.end())); 
    int miny = round( *std::min_element(oy.begin(), oy.end())); 
    int maxx = round( *std::max_element(ox.begin(), ox.end())); 
    int maxy = round( *std::max_element(oy.begin(), oy.end()));
    int xwidth = round(maxx - maxy); 
    int ywidth = round(maxy - miny); 
    std::vector<std::vector<bool>> obmap; 
    for (int i = 0; i < xwidth; i++)
        for (int j = 0; j < ywidth; j++)
            obmap[i][j] = false;
    
}