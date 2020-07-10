#include<cmath>
#include<vector> 

using namespace::std; 

class T{ 
    public: 
        T(float lw = 19, float sw = 4, float trans_l = 4, float tl = 8): large_w(lw), short_w(sw), transition_l(trans_l), total_l(tl) {};
    private:
        float large_w; 
        float short_w; 
        float transition_l; 
        float total_l;
};

class V{ 
    public: 
        V(float long_wing_span = 18, float short_wing_span = 3, float body_span = 2, float length = 7, float wheel_base = 12, float max_steering_angle = 0.6, float min_turning_radius = 5): wing_span(long_wing_span), short_span(short_wing_span), body_span(body_span), total_length(total_length), wheel_base(wheel_base), max_steering_angle(max_steering_angle), minimum_turning_radius(min_turning_radius) {}
    private:
        float wing_span; 
        float short_span; 
        float body_span; 
        float total_length; 
        int wheel_base; 
        float overhang; 
        float max_steering_angle; 
        float minimum_turning_radius; 
        float inner_turning_radius = sqrt(pow(minimum_turning_radius, 2) - pow(wheel_base, 2) - (body_span / 2)); 
        float outer_turning_radius = sqrt(pow(inner_turning_radius + body_span, 2) + pow(wheel_base + overhang, 2)); 
        float min_depth = overhang + sqrt(pow(outer_turning_radius, 2) + pow(inner_turning_radius, 2));
};

class ParkPoints{ 
    public: 
        ParkPoints(float r_p, float c1, float c2, float i, float s, float pt): r_prime(r_prime), c1(c1), c2(c2), i(i), s(s), pt(pt) {};
    private: 
        float r_prime; 
        float c1; 
        float c2; 
        float i; 
        float s; 
        float pt;
};

class Directions{ 
    public:
        Directions(float vel, float steer): vel(vel), steer(steer) {}; 
    private: 
        float vel; 
        float steer;
};

class Parking: public V, public T{
    public: 
        Parking(float m): margin(m){};
    private:
        float d(std::vector<float> a, std::vector<float> b); 
        ParkPoints park_points(float ri, std::vector<float> current, std::vector<float> goal, float w); 
        Directions veh_control(std::vector<float> initial_pos, std::vector<float> start_pos, std::vector<float> goal_pos, std::vector<float> update_pos, float orientation); 
        float margin;
};

float Parking::d(std::vector<float> a, std::vector<float> b){ 
    return sqrt(pow(a[0] - b[0],2) + pow(a[1] - b[1], 2));
}; 

ParkPoints Parking::
