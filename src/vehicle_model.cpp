#include "vehicle_model.hpp"

#include <algorithm>
#include <cmath>

struct StateDerivative {
    double dx;
    double dy;
    double dv;
    double dheading;
};

static StateDerivative derivative(
    const VehicleState& state,
    const Scenario& scenario,
    double steeringAngle
) {
    StateDerivative d{};

    d.dx = state.velocity * std::cos(state.heading);
    d.dy = state.velocity * std::sin(state.heading);
    d.dv = scenario.acceleration;

    d.dheading =
        (state.velocity / scenario.wheelbase)
        * std::tan(steeringAngle);

    return d;
}


void updateVehicleEuler(
    VehicleState& vehicle,
    const Scenario& scenario,
    double steeringAngle
) {
    const StateDerivative d =
        derivative(vehicle, scenario, steeringAngle);

    vehicle.x += d.dx * scenario.dt;
    vehicle.y += d.dy * scenario.dt;
    vehicle.velocity += d.dv * scenario.dt;
    vehicle.heading += d.dheading * scenario.dt;

    vehicle.velocity =
        std::max(0.0, vehicle.velocity);
}


void updateVehicleRK4(
    VehicleState& vehicle,
    const Scenario& scenario,
    double steeringAngle
) {
    const double dt = scenario.dt;

    const StateDerivative k1 =
        derivative(vehicle, scenario, steeringAngle);

    VehicleState s2{
        vehicle.x + 0.5 * dt * k1.dx,
        vehicle.y + 0.5 * dt * k1.dy,
        vehicle.velocity + 0.5 * dt * k1.dv,
        vehicle.heading + 0.5 * dt * k1.dheading
    };

    const StateDerivative k2 =
        derivative(s2, scenario, steeringAngle);

    VehicleState s3{
        vehicle.x + 0.5 * dt * k2.dx,
        vehicle.y + 0.5 * dt * k2.dy,
        vehicle.velocity + 0.5 * dt * k2.dv,
        vehicle.heading + 0.5 * dt * k2.dheading
    };

    const StateDerivative k3 =
        derivative(s3, scenario, steeringAngle);

    VehicleState s4{
        vehicle.x + dt * k3.dx,
        vehicle.y + dt * k3.dy,
        vehicle.velocity + dt * k3.dv,
        vehicle.heading + dt * k3.dheading
    };

    const StateDerivative k4 =
        derivative(s4, scenario, steeringAngle);

    vehicle.x +=
        dt / 6.0
        * (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx);

    vehicle.y +=
        dt / 6.0
        * (k1.dy + 2.0 * k2.dy + 2.0 * k3.dy + k4.dy);

    vehicle.velocity +=
        dt / 6.0
        * (k1.dv + 2.0 * k2.dv + 2.0 * k3.dv + k4.dv);

    vehicle.heading +=
        dt / 6.0
        * (
            k1.dheading
            + 2.0 * k2.dheading
            + 2.0 * k3.dheading
            + k4.dheading
        );

    vehicle.velocity =
        std::max(0.0, vehicle.velocity);
}


void updateVehicle(
    VehicleState& vehicle,
    const Scenario& scenario,
    double steeringAngle,
    IntegrationMethod method
) {
    if (method == IntegrationMethod::RK4) {
        updateVehicleRK4(
            vehicle,
            scenario,
            steeringAngle
        );
    } else {
        updateVehicleEuler(
            vehicle,
            scenario,
            steeringAngle
        );
    }
}