#ifndef KALMAN_HPP
#define KALMAN_HPP

class KalmanFilter1D {
private:
    float q; // Шум процесу (Process noise)
    float r; // Шум вимірювання (Measurement noise)
    float x; // Оцінене значення
    float p; // Помилка оцінки
    float k; // Коефіцієнт Калмана

public:
    KalmanFilter1D(float process_noise = 0.01f, float measurement_noise = 0.25f, float estimation_error = 1.0f) 
        : q(process_noise), r(measurement_noise), p(estimation_error), x(0.0f), k(0.0f) {}

    void init(float initial_value) {
        x = initial_value;
    }

    float update(float measurement) {
        p = p + q;
        k = p / (p + r);
        x = x + k * (measurement - x);
        p = (1.0f - k) * p;
        return x;
    }

    float get_value() const { return x; }
};

#endif // KALMAN_HPP
