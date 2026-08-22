#include <cassert>
#include <cmath>
#include <iostream>

#include "vehicle_model.hpp"


void testAcceleration() {
    Scenario scenario{};

    scenario.acceleration = 2.0;
    scenario.dt = 0.5;
    scenario.wheelbase = 2.7;

    VehicleState car{
        0.0,
        0.0,
        5.0,
        0.0
    };

    updateVehicle(
        car,
        scenario,
        0.0,
        IntegrationMethod::Euler
    );

    assert(
        std::abs(car.velocity - 6.0) < 1e-9
    );

    std::cout
        << "testAcceleration PASSED\n";
}


void testBrakingDoesNotGoNegative() {
    Scenario scenario{};

    scenario.acceleration = -20.0;
    scenario.dt = 1.0;
    scenario.wheelbase = 2.7;

    VehicleState car{
        0.0,
        0.0,
        5.0,
        0.0
    };

    updateVehicle(
        car,
        scenario,
        0.0,
        IntegrationMethod::Euler
    );

    assert(car.velocity >= 0.0);
    assert(std::abs(car.velocity) < 1e-9);

    std::cout
        << "testBrakingDoesNotGoNegative PASSED\n";
}


void testStraightDriving() {
    Scenario scenario{};

    scenario.acceleration = 0.0;
    scenario.dt = 0.1;
    scenario.wheelbase = 2.7;

    VehicleState car{
        0.0,
        0.0,
        10.0,
        0.0
    };

    updateVehicle(
        car,
        scenario,
        0.0,
        IntegrationMethod::Euler
    );

    assert(std::abs(car.y) < 1e-9);
    assert(std::abs(car.heading) < 1e-9);

    std::cout
        << "testStraightDriving PASSED\n";
}


void testPositiveSteeringChangesHeading() {
    Scenario scenario{};

    scenario.acceleration = 0.0;
    scenario.dt = 0.1;
    scenario.wheelbase = 2.7;

    VehicleState car{
        0.0,
        0.0,
        10.0,
        0.0
    };

    updateVehicle(
        car,
        scenario,
        0.1,
        IntegrationMethod::Euler
    );

    assert(car.heading > 0.0);

    std::cout
        << "testPositiveSteeringChangesHeading PASSED\n";
}


void testRK4StraightDriving() {
    Scenario scenario{};

    scenario.acceleration = 0.0;
    scenario.dt = 0.1;
    scenario.wheelbase = 2.7;

    VehicleState car{
        0.0,
        0.0,
        10.0,
        0.0
    };

    updateVehicle(
        car,
        scenario,
        0.0,
        IntegrationMethod::RK4
    );

    assert(std::abs(car.y) < 1e-9);
    assert(std::abs(car.heading) < 1e-9);
    assert(std::abs(car.x - 1.0) < 1e-9);

    std::cout
        << "testRK4StraightDriving PASSED\n";
}


void testRK4PositiveSteeringChangesHeading() {
    Scenario scenario{};

    scenario.acceleration = 0.0;
    scenario.dt = 0.1;
    scenario.wheelbase = 2.7;

    VehicleState car{
        0.0,
        0.0,
        10.0,
        0.0
    };

    updateVehicle(
        car,
        scenario,
        0.1,
        IntegrationMethod::RK4
    );

    assert(car.heading > 0.0);
    assert(car.y > 0.0);

    std::cout
        << "testRK4PositiveSteeringChangesHeading PASSED\n";
}


int main() {
    testAcceleration();
    testBrakingDoesNotGoNegative();
    testStraightDriving();
    testPositiveSteeringChangesHeading();

    testRK4StraightDriving();
    testRK4PositiveSteeringChangesHeading();

    std::cout
        << "\nAll vehicle model tests passed.\n";

    return 0;
}