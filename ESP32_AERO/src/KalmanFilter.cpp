#include "KalmanFilter.h"
#include <cmath>

KalmanFilter1D::KalmanFilter1D(float process_noise, float sensor_noise, float initial_value)
    : q(process_noise), r(sensor_noise), x(initial_value), p(1.0f), k(0.0f) {}

float KalmanFilter1D::update(float measurement) {
    if (std::isnan(measurement)) return x;
    // Prediction
    p = p + q;
    // Update
    k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1.0f - k) * p;
    return x;
}

void KalmanFilter1D::setState(float new_x) {
    x = new_x;
}

float KalmanFilter1D::getState() const { return x; }
