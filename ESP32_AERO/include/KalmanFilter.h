#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <cmath> // For isnan

// Simple 1D Kalman Filter for sensor smoothing
class KalmanFilter1D {
public:
    KalmanFilter1D(float process_noise, float sensor_noise, float initial_value);

    float update(float measurement);

    void setState(float new_x);
    float getState() const;

private:
    float q, r, x, p, k;
};

#endif // KALMAN_FILTER_H
