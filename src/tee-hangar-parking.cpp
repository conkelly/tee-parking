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
        float get_wing_span(){ 
            return wing_span; };
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
        ParkPoints(float r_p, std::vector<float> c1, std::vector<float> c2, std::vector<float> i, std::vector<float> s, std::vector<float> pt): r_prime(r_prime), c1(c1), c2(c2), i(i), s(s), pt(pt) {};
    private: 
        float r_prime; 
        std::vector<float> c1; 
        std::vector<float> c2; 
        std::vector<float> i; 
        std::vector<float> s; 
        std::vector<float> pt;
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

ParkPoints Parking::park_points(float ri, std::vector<float> current, std::vector<float> goal, float w){ 
    float r_prime = ri + w/2; 
    std::vector<float> c1;
    c1[0] = goal[0]; 
    c1[1] = goal[1];
    float x_c1 = c1[0]; 
    float y_c1 = c1[1]; 
    float x_i = current[0]; 
    float y_i = current[1]; 
    float y_s = y_i; 
    float y_c2 = y_s - r_prime; 
    float y_t = (y_c1 + y_c2) / 2; 
    float x_t = x_c1 + sqrt(pow(r_prime, 2) - pow(y_t - y_c1, 2)); 
    float x_s = 2 * x_t - x_c1; 
    float x_c2 = x_s; 
    std::vector<float> c2; 
    c2[0] = x_c2; 
    c2[1 ]= y_c2; 
    std::vector<float> i; 
    i[0] = x_i; 
    i[1] = y_i; 
    std::vector<float> s; 
    s[0] = x_s; 
    s[1] = y_s;
    std::vector<float> pt; 
    pt[0] = x_t; 
    pt[1] = y_t; 
    ParkPoints park(r_prime, c1, c2, i, s, pt); 
    return park; 
}; 

Directions Parking::veh_control(std::vector<float> initial_pos, std::vector<float> start_pos, std::vector<float> goal_pos, std::vector<float> update_pos, float orientation){ 
    Directions out(0,0);
    //Here we are assuming wing span is greater than turning radius.
    if (update_pos[1] + Parking::get_wing_span() / 2 > start_pos[1]){ 
        if (orientation == 90){ 
            Directions out(12, 0); // turn, need minimum turnign radius.
        } else { 
            Directions out(1, 5);
        }
    }
    else if(update_pos[0] > start_pos[0]){ 
        if (orientation == 180){
            Directions out(12, 0); //turn around, need minimum turning radius. 
        } else{ 
            Directions out(1,5);
        }
    }
    
    else if (update_pos[0] < start_pos[0]) { 
        if (orientation == 0){ 
            Directions out(12, 0);
        } else { 
            Directions out(1, 5);
        }
    }
    // Unsure how to implement turning radius here.
    //Here because this is a positional argument we are selecting the axis such that update_pos[1] < goal_pos[1]
    else if((update_pos[0] == start_pos[0])){ 
        if (orientation == 270){ 
            Directions out(12, 0); 
        } else { 
            Directions out(1, 5);
        }
    }

    
    return out;
}

//Run through functions with establish params and then send to publishers.