#include <iostream>


#include "rl_trajectory_generation.cpp"

int main(int argc, char **argv) {
    TrajectoryGenerator gen;
    auto start = high_resolution_clock::now();
    double position[] {1, 0, 0};
    double linear_velocity[] {0, 0, 0};
    double rotation[] {1, 0, 0, 0};
    double angular_velocity[] {1, 0, 0};
    double target[] {0, 0, 0};
    for(int i = 0; i < 100; i++) {
     gen.generateTrajectory(
        position, // Initial Position
        linear_velocity, // linear velocity
        rotation, // rotation
        angular_velocity, // angular velocity
        target, // target
        0.02, // dt
        1 // number of seconds
        );
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    cout << duration.count()/100. << endl; 
    
    return 0;
}

