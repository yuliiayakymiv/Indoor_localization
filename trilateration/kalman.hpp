#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <Eigen/Dense>

class KalmanFilter3D {
public:
    using StateVector = Eigen::Matrix<float, 6, 1>;
    using MeasVector  = Eigen::Matrix<float, 3, 1>;
    using Matrix6x6   = Eigen::Matrix<float, 6, 6>;
    using Matrix3x6   = Eigen::Matrix<float, 3, 6>;
    using Matrix6x3   = Eigen::Matrix<float, 6, 3>;
    using Matrix3x3   = Eigen::Matrix<float, 3, 3>;

    KalmanFilter3D() : is_initialized_(false) {
        x_.setZero();
        P_.setIdentity();

        //Матриця вимірювань
        H_.setZero();
        H_(0, 0) = 1.0f;
        H_(1, 1) = 1.0f;
        H_(2, 2) = 1.0f;

        //Коваріація шуму вимірювань
        R_ = Matrix3x3::Identity() * 0.05f; // ~5 см дисперсія вимірювання

        //Коваріація шуму процесу
        Q_ = Matrix6x6::Identity() * 0.01f;
    }

    void init(float x, float y, float z) {
        x_.setZero();
        x_(0) = x;
        x_(1) = y;
        x_(2) = z;
        P_ = Matrix6x6::Identity() * 0.1f;
        is_initialized_ = true;
    }

    //Prediction
    void predict(float dt) {
        if (!is_initialized_) return;

        Matrix6x6 F = Matrix6x6::Identity();
        F(0, 3) = dt;
        F(1, 4) = dt;
        F(2, 5) = dt;

        // x = F * x
        x_ = F * x_;
        // P = F * P * F^T + Q
        P_ = F * P_ * F.transpose() + Q_;
    }

    // Етап Оновлення (Update)
    void update(float x_m, float y_m, float z_m) {
        if (!is_initialized_) {
            init(x_m, y_m, z_m);
            return;
        }

        MeasVector z;
        z << x_m, y_m, z_m;

        //Infiltration error
        MeasVector y = z - H_ * x_;

        //Innovation covariance
        Matrix3x3 S = H_ * P_ * H_.transpose() + R_;

        //Kalman gain
        Matrix6x3 K = P_ * H_.transpose() * S.inverse();

        //New state
        x_ = x_ + K * y;

        //New covariance
        Matrix6x6 I = Matrix6x6::Identity();
        P_ = (I - K * H_) * P_;
    }

    float getX() const { return x_(0); }
    float getY() const { return x_(1); }
    float getZ() const { return x_(2); }
    float getVx() const { return x_(3); }
    float getVy() const { return x_(4); }
    float getVz() const { return x_(5); }

    bool isInitialized() const { return is_initialized_; }

private:
    StateVector x_;
    Matrix6x6 P_;        //Коваріація помилки
    Matrix3x6 H_;        //Матриця спостереження
    Matrix3x3 R_;        //Шум вимірювань
    Matrix6x6 Q_;        //Шум фізичної моделі
    bool is_initialized_;
};

#endif // KALMAN_HPP
