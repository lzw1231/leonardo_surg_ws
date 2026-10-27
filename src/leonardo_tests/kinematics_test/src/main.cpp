#include "kinematics_test/surgical_kinematics.hpp"
#include <iostream>

int main() {
    kinematics_test::Post post;
    kinematics_test::Dof dof;
    kinematics_test::SurgicalKinematics solver;
    bool ok = solver.solvePsm(post, dof);
    std::cout << "solvePsm ok=" << ok << '\n';
    return ok ? 0 : 1;
}