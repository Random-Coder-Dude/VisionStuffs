#include <stdio.h>
#include <math.h>

#define SIGMA_MIN 1e-6

struct Vector3d {
    double x;
    double y;
    double z;
};

static double normalCDF(double x) {
    return 0.5 * (1.0 + erf(x / sqrt(2.0)));
}

double calculateShotForce(double transferEfficiency, double ballMass, double shooterRPM,
                          double bottomWheelRadius, double gearRatio, double topWheelRadius,
                          double compression, double compressionRatio, double contactTime,
                          double currentTime) {

    if (currentTime < 0.0 || currentTime > contactTime) return 0.0;

    double omega_b    = (2.0 * M_PI * shooterRPM) / 60.0;
    double v_rim_bot  = omega_b * bottomWheelRadius;
    double v_rim_top  = omega_b * gearRatio * topWheelRadius;

    double v_muzzle   = (v_rim_bot + v_rim_top) / 2.0;

    double impulse    = transferEfficiency * ballMass * v_muzzle;

    double t0         = contactTime / 2.0;
    double sigma      = fmax(compressionRatio * compression, SIGMA_MIN);

    double cdf_lo     = normalCDF((0.0         - t0) / sigma);
    double cdf_hi     = normalCDF((contactTime  - t0) / sigma);
    double norm       = cdf_hi - cdf_lo;

    if (norm < 1e-12) return 0.0;

    double exponent   = -0.5 * pow((currentTime - t0) / sigma, 2);
    double gaussian   = (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(exponent);

    return (impulse / norm) * gaussian;
}

int main() {
    return 0;
}