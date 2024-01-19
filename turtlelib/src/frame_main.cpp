#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
#include <fstream>

int main(){
    using namespace turtlelib;
    Drawer svg("/tmp/frames.svg");

    // Inputting Transforms
    Transform2D T_ab, T_bc, T_ac, T_cb, T_ba, T_ca; 
    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> T_ab;
    
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> T_bc;


    // Calculates Frames
    std::cout << "T_{a,b}: " << T_ab << std::endl;

    T_ba = T_ab.inv();
    std::cout << "T_{b,a}: " << T_ba << std::endl;

    std::cout << "T_{b,c}: " << T_bc << std::endl;

    T_cb = T_bc.inv();
    std::cout << "T_{c,b}: " << T_cb << std::endl;

    T_ac = T_ab * T_bc;
    std::cout << "T_{a,c}: " << T_ac << std::endl;

    T_ca = T_ac.inv();
    std::cout << "T_{c,a}: " << T_ca << std::endl;


    // Drawing frames
    Point2D origin = {0.0, 0.0};
    Point2D x = {1.0, 0.0};
    Point2D y = {0.0, 1.0};

    // Frame A
    svg.frame(origin, x, y, "a", {-0.2, -0.2});
    
    // Frame B
    Point2D originB = T_ab(origin);
    Point2D xB = T_ab(x);
    Point2D yB = T_ab(y);
    svg.frame(originB, xB, yB, "b", {originB.x-0.2, originB.y-0.2});

    // Frame C
    Point2D originC = T_ac(origin);
    Point2D xC = T_ac(x);
    Point2D yC = T_ac(y);
    svg.frame(originC, xC, yC, "c", {originC.x-0.2, originC.y-0.2});
    

    Point2D p_a, p_b, p_c;

    // Point A
    std::cout << "Enter point p_a:" << std::endl;
    std::cin >> p_a;
    std::cout << "p_a: " << p_a << std::endl;
    svg.circle(p_a, "purple", "purple");

    // Point B
    p_b = T_ba(p_a);
    std::cout << "p_b: " << p_b << std::endl;
    svg.circle(p_b, "brown", "brown");

    // Point C
    p_c = T_ac(p_a);
    std::cout << "p_c: " << p_c << std::endl;
    svg.circle(p_c, "orange", "orange");
    
    // Vector B
    Vector2D v_a, v_b, v_c, v_bhat; 
    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> v_b;
    v_bhat = normalize(v_b);
    std::cout << "v_bhat: " << v_bhat << std::endl;
    svg.vector({v_bhat.x+originB.x, v_bhat.y+originB.y}, originB, "brown");

    // Vector A
    v_a = T_ab(v_b);
    std::cout << "v_a: " << v_a << std::endl;
    svg.vector({v_a.x+origin.x, v_a.y+origin.y}, origin, "purple");

    std::cout << "v_b: " << v_b << std::endl;

    // Vector C
    v_c = T_cb(v_b);
    std::cout << "v_c: " << v_c << std::endl;
    svg.vector({v_c.x+originC.x, v_c.y+originC.y}, originC, "orange");


    // Twist B
    Twist2D tw_a, tw_b, tw_c;
    std::cout << "Enter a twist V_b:" << std::endl;
    std::cin >> tw_b;

    // Twist A
    tw_a = T_ab(tw_b);
    std::cout << "V_a: " << tw_a << std::endl;

    std::cout << "V_b: " << tw_b << std::endl;

    // Twist C
    tw_c = T_cb(tw_b);
    std::cout << "V_c: " << tw_c << std::endl;


    svg.save();
    return 0;
}