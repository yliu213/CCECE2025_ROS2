#include "Eigen/Dense"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#define PI 3.14159265358979323846  // Define PI

template <typename Type>
class SecondOrderFilter {
public:
    // Constructor
    SecondOrderFilter(double cutoff_freq, double Q, bool verbose) : verbose_(verbose) {
        if (verbose_) RCLCPP_INFO(rclcpp::get_logger("SecondOrderFilter"), "[2ndFilter] Construction Started");

        omega_ = 2 * PI * cutoff_freq; // cutoff frequency
        xi_ = 1 / (2 * Q); // damping ratio

        if (verbose_) RCLCPP_INFO(rclcpp::get_logger("SecondOrderFilter"), "[2ndFilter] Construction Complete");
    }

    ~SecondOrderFilter() {
        if (verbose_) RCLCPP_INFO(rclcpp::get_logger("SecondOrderFilter"), "[2ndFilter] Destructed");
    }

    Type updateFilter(Type x_k, double sampling_time) {
        const double omega = omega_;
        const double xi = xi_;
        const double T = sampling_time;

        // Filter coefficients
        const double k1 = omega * omega * T * T + 4 * omega * xi * T + 4;
        const double k2 = 2 * omega * omega * T * T - 8;
        const double k3 = omega * omega * T * T - 4 * omega * xi * T + 4;
        const double k4 = omega * omega * T * T;
        const double k5 = 2 * k4;
        const double k6 = k4;

        a_[0] = 1;
        a_[1] = k2 / k1;
        a_[2] = k3 / k1;
        b_[0] = k4 / k1;
        b_[1] = k5 / k1;
        b_[2] = k6 / k1;

        if (verbose_) {
            RCLCPP_INFO(rclcpp::get_logger("SecondOrderFilter"), 
                "[filterUpdate] T=%.6f omega=%.6f xi=%.6f b=[%.6f, %.6f, %.6f] a=[%.6f, %.6f, %.6f]", 
                T, omega, xi, b_[0], b_[1], b_[2], a_[0], a_[1], a_[2]);
        }

        // Discrete-time 2nd order filter
        Type y_k;
        y_k = b_[0] * x_k + b_[1] * x_k_1_ + b_[2] * x_k_2_ - a_[1] * y_k_1_ - a_[2] * y_k_2_;

        // Shift previous values for next iteration
        x_k_2_ = x_k_1_;
        x_k_1_ = x_k;
        y_k_2_ = y_k_1_;
        y_k_1_ = y_k;

        return y_k;
    }

private:
    double b_[3];  // Coefficients of numerator
    double a_[3];  // Coefficients of denominator
    Type x_k_1_, x_k_2_;  // x(k-1), x(k-2) inputs
    Type y_k_1_, y_k_2_;  // y(k-1), y(k-2) filtered outputs
    double omega_;
    double xi_;
    bool verbose_;

protected:
};


