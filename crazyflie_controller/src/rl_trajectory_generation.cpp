
#include <iostream>
#include <Python.h>
#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double, Eigen::Dynamic, 18> TrajectoryMatrix;

#include <chrono> 
using namespace std::chrono;
using namespace std;

namespace py = pybind11;


class TrajectoryGenerator {
private:
    py::scoped_interpreter guard;
    py::object python_generator_function;
public:
    TrajectoryGenerator() {
        python_generator_function = py::module::import("rl.rl_trajectory_generator").attr("make_trajectory_generator")();
    }

    TrajectoryMatrix generateTrajectory(
        double initial_position[],
        double initial_linear_velocity[],
        double initial_rotation[],
        double initial_angular_velocity[],
        double target[],
        float dt,
        float n_seconds
        ) 
    {
        py::array trajectory = python_generator_function(
            py::array_t<double>(3, initial_position),
            py::array_t<double>(3, initial_linear_velocity),
            py::array_t<double>(4, initial_rotation),
            py::array_t<double>(3, initial_angular_velocity),
            py::array_t<double>(3, target),
            dt,
            n_seconds
            );
        // std::cout << trajectory.shape()[1] << std::endl;
        TrajectoryMatrix matrix_trajectory(trajectory.shape()[0], trajectory.shape()[1]);
        for(int t = 0; t < trajectory.shape()[0]; ++t) {
            for(int i = 0; i < 18; ++i) {
                matrix_trajectory(t, i) = *((double*)trajectory.data(t, i));    
            }
        }
        return matrix_trajectory;
    }
};


void example()
{
    py::scoped_interpreter guard{};
    std::cout << "rl trajectory example" << std::endl;

    py::object make_trajectory_generator = py::module::import("rl.rl_trajectory_generator").attr("make_trajectory_generator");
    py::object trajectory_generator = make_trajectory_generator();

    trajectory_generator();
    trajectory_generator();
    trajectory_generator();
    auto start = high_resolution_clock::now();
    vector<double> initial_position{1, 0, 0};
    for(int i = 0; i < 100; i++) {
     trajectory_generator(
        // vector<double>{1, 0, 0}
        py::array_t<double>(initial_position),
        py::array_t<double>({0., 0., 0.})
        // py::array_t<double>{1, 0, 0, 0}, 
        // py::array_t<double>{0, 0, 0}, 
        // py::array_t<double>{0, 0, 0}, 
        // 0.02, 
        // 1,
        // initial_propeller_speed: np.ndarray=None):
        );
    }
    auto stop = high_resolution_clock::now();

    auto duration = duration_cast<milliseconds>(stop - start);
    cout << duration.count()/100. << endl; 

    
}
