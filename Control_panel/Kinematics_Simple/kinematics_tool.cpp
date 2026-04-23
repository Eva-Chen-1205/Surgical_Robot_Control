#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <string>

// ============== Math Helpers ==============
const double PI = 3.14159265358979323846;

double deg2rad(double deg) { return deg * PI / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / PI; }
double sind(double deg) { return std::sin(deg2rad(deg)); }
double cosd(double deg) { return std::cos(deg2rad(deg)); }
double tand(double deg) { return std::tan(deg2rad(deg)); }
double asind(double val) { return rad2deg(std::asin(val)); }
double acosd(double val) { return rad2deg(std::acos(val)); }
double atand(double val) { return rad2deg(std::atan(val)); }

struct Pose {
    double yaw, pitch, a_z, a_y, u5;
};

struct Joints {
    double theta1, theta2, theta3, theta4, theta5;
};

// ============== Inverse Kinematics ==============
Joints inverse_kinematics(Pose p) {
    Joints y = {0,0,0,0,0};
    double Yaw = p.yaw;
    double Pitch = p.pitch;
    double A_z = p.a_z;
    double A_y = p.a_y;
    double B_z = 0; double B_y = 0;
    double C_z = 20; double C_y = 0;
    double r1 = 55; double r2 = 77.2;
    double AB = std::sqrt(std::pow(B_z - A_z, 2) + std::pow(B_y - A_y, 2));
    double AC = std::sqrt(std::pow(C_z - A_z, 2) + std::pow(C_y - A_y, 2));

    auto safe_atand = [](double dy, double dx) {
        if (std::abs(dx) < 1e-9) return dy > 0 ? 90.0 : -90.0;
        return rad2deg(std::atan(dy / dx));
    };

    double AB_angle = -safe_atand((B_y - A_y), (B_z - A_z));
    double angle1 = std::abs(Yaw - AB_angle);
    double sin_phi1 = AB * std::abs(sind(angle1) / r1);
    if (sin_phi1 > 1.0) sin_phi1 = 1.0;
    double phi1 = asind(sin_phi1);
    double DD_1 = std::abs(r1 / std::abs(sind(angle1)) * sind(180 - angle1 - phi1));
    if (std::isnan(DD_1) || std::isinf(DD_1)) DD_1 = AB + r1;
    
    double r1_z = DD_1 * cosd(Yaw) - (B_z - A_z);
    double r1_y = -DD_1 * sind(Yaw) - (B_y - A_y);
    y.theta1 = -std::atan2(r1_y, r1_z);

    double AC_angle = -safe_atand((C_y - A_y), (C_z - A_z));
    double angle2 = std::abs(Yaw - AC_angle);
    double sin_phi2 = AC * std::abs(sind(angle2) / r2);
    if (sin_phi2 > 1.0) sin_phi2 = 1.0;
    double phi2 = asind(sin_phi2);
    double DD_2 = std::abs(r2 / std::abs(sind(angle2)) * sind(180 - angle2 - phi2));
    if (std::isnan(DD_2) || std::isinf(DD_2)) DD_2 = AC + r2;
    double r2_z = DD_2 * cosd(Yaw) - (C_z - A_z);
    double r2_y = -DD_2 * sind(Yaw) - (C_y - A_y);
    y.theta2 = -std::atan2(r2_y, r2_z);

    double x_dim = 25;
    double h1 = DD_1 - 30;
    double d_total = p.u5 - 74.2;
    double d_2 = x_dim * cosd(90.0 - Pitch) + (x_dim * sind(90.0 - Pitch) + h1) * cosd(Pitch)/sind(Pitch);
    if (std::isnan(d_2)) d_2 = 0;
    double d_1 = d_total - d_2;

    double hart1 = 18; double hart2 = 15.05;
    double thetah_ori = acosd(10.0 / 50.0) + 90.0;
    double hart3_ori = std::sqrt(std::pow(hart1,2) + std::pow(hart2,2) - 2 * hart1 * hart2 * cosd(thetah_ori));
    double val_acos = (d_1 + 20) / 100.0;
    if (val_acos > 1.0) val_acos = 1.0;
    if (val_acos < -1.0) val_acos = -1.0;
    double thetah_after = acosd(val_acos) + 90.0;
    double hart3_after = std::sqrt(std::pow(hart1,2) + std::pow(hart2,2) - 2 * hart1 * hart2 * cosd(thetah_after));
    
    double sin_t3_ori = hart2 / hart3_ori * sind(thetah_ori);
    if (sin_t3_ori > 1.0) sin_t3_ori = 1.0;
    double t3_ori = asind(sin_t3_ori);
    double theta3_ori = 2 * t3_ori;
    
    double sin_t3_after = hart2 / hart3_after * sind(thetah_after);
    if (sin_t3_after > 1.0) sin_t3_after = 1.0;
    double t3_after = asind(sin_t3_after);
    double theta3_after = 2 * t3_after;
    y.theta3 = (theta3_after - theta3_ori) * PI / 180.0;

    double theta5_ori = 46.5;
    double theta5 = h1 / cosd(90.0 - Pitch) + x_dim * tand(90.0 - Pitch);
    y.theta5 = (theta5 - theta5_ori) * PI;

    double L1 = 66 + d_1;
    double h = 18;
    double thetaa = Pitch + 21;
    double R = 50;
    double A = 76.3;
    double B = 98.07;
    double beta = safe_atand(L1, h);
    double alpha = safe_atand(h, L1);
    double c = std::sqrt((std::pow(L1,2) + std::pow(h,2)) + std::pow(R,2) - 2 * std::sqrt(std::pow(L1,2) + std::pow(h,2)) * R * cosd(thetaa - alpha));
    
    auto safe_acosd = [](double v) {
        if (v > 1.0) return 0.0;
        if (v < -1.0) return 180.0;
        return rad2deg(std::acos(v));
    };

    double cos_theta4_2 = (std::pow(A,2) + std::pow(c,2) - std::pow(B,2)) / (2 * A * c);
    double cos_theta4_1 = (std::pow(h,2) + std::pow(L1,2) + std::pow(c,2) - std::pow(R,2)) / (2 * std::sqrt(std::pow(h,2) + std::pow(L1,2)) * c);
    double theta4_1 = safe_acosd(cos_theta4_1);
    double theta4_2 = safe_acosd(cos_theta4_2);
    double theta4 = 270 - theta4_1 - theta4_2 - beta;

    double L1_ori = 66;
    double theta_ori = 70 + 21;
    double beta_ori = safe_atand(L1_ori, h);
    double alpha_ori = safe_atand(h, L1_ori);
    double c_ori = std::sqrt((std::pow(L1_ori,2) + std::pow(h,2)) + std::pow(R,2) - 2 * std::sqrt(std::pow(L1_ori,2) + std::pow(h,2)) * R * cosd(theta_ori - alpha_ori));
    double cos_theta4_2_ori = (std::pow(A,2) + std::pow(c_ori,2) - std::pow(B,2)) / (2 * A * c_ori);
    double cos_theta4_1_ori = (std::pow(h,2) + std::pow(L1_ori,2) + std::pow(c_ori,2) - std::pow(R,2)) / (2 * std::sqrt(std::pow(h,2) + std::pow(L1_ori,2)) * c_ori);
    double theta4_1_ori = safe_acosd(cos_theta4_1_ori);
    double theta4_2_ori = safe_acosd(cos_theta4_2_ori);
    double theta4_ori = 270.0 - theta4_1_ori - theta4_2_ori - beta_ori;
    y.theta4 = (theta4 - theta4_ori) * PI / 180.0;

    return y;
}

// ============== Numerical Forward Kinematics ==============
std::vector<double> solve_linear_system(std::vector<std::vector<double>> A, std::vector<double> b) {
    int n = (int)b.size();
    for (int i = 0; i < n; i++) {
        int max_el = i;
        for (int k = i + 1; k < n; k++) {
            if (std::abs(A[k][i]) > std::abs(A[max_el][i])) max_el = k;
        }
        std::swap(A[max_el], A[i]);
        std::swap(b[max_el], b[i]);
        double pivot = A[i][i];
        if (std::abs(pivot) < 1e-9) pivot = pivot > 0 ? 1e-9 : -1e-9;
        for (int k = i + 1; k < n; k++) {
            double c = -A[k][i] / pivot;
            for (int j = i; j < n; j++) {
                if (i == j) A[k][j] = 0;
                else A[k][j] += c * A[i][j];
            }
            b[k] += c * b[i];
        }
    }
    std::vector<double> x(n);
    for (int i = n - 1; i >= 0; i--) {
        x[i] = b[i] / A[i][i];
        for (int k = i - 1; k >= 0; k--) b[k] -= A[k][i] * x[i];
    }
    return x;
}

Pose forward_kinematics(const Joints& target) {
    std::vector<double> tgt = { target.theta1, target.theta2, target.theta3, target.theta4, target.theta5 };
    std::vector<double> u = { 0, 90, -18, 0, 140 }; // Initial guess
    
    double lambda = 1e-3;
    for (int iter = 0; iter < 200; iter++) {
        Pose p = {u[0], u[1], u[2], u[3], u[4]};
        Joints y = inverse_kinematics(p);
        std::vector<double> current_y = { y.theta1, y.theta2, y.theta3, y.theta4, y.theta5 };
        
        std::vector<double> err(5);
        double max_err = 0;
        for(int i=0; i<5; i++) {
            err[i] = current_y[i] - tgt[i];
            if (std::abs(err[i]) > max_err) max_err = std::abs(err[i]);
        }
        
        if (max_err < 1e-7) return p;

        std::vector<std::vector<double>> J(5, std::vector<double>(5));
        double delta = 1e-6;
        for (int j = 0; j < 5; j++) {
            std::vector<double> u_plus = u; u_plus[j] += delta;
            Pose p_plus = {u_plus[0], u_plus[1], u_plus[2], u_plus[3], u_plus[4]};
            Joints y_plus = inverse_kinematics(p_plus);
            std::vector<double> yp = { y_plus.theta1, y_plus.theta2, y_plus.theta3, y_plus.theta4, y_plus.theta5 };
            for(int i=0; i<5; i++) J[i][j] = (yp[i] - current_y[i]) / delta;
        }

        std::vector<std::vector<double>> JTJ(5, std::vector<double>(5, 0));
        std::vector<double> JTe(5, 0);
        for(int i=0; i<5; i++) {
            for(int j=0; j<5; j++) {
                for(int k=0; k<5; k++) JTJ[i][j] += J[k][i] * J[k][j];
                JTe[i] += J[j][i] * err[j];
            }
        }
        for(int i=0; i<5; i++) JTJ[i][i] += lambda;
        for(int i=0; i<5; i++) JTe[i] = -JTe[i];
        
        std::vector<double> du = solve_linear_system(JTJ, JTe);
        for(int i=0; i<5; i++) u[i] += du[i];
    }
    return {u[0], u[1], u[2], u[3], u[4]};
}

// ============== Interactive CLI ==============
int main() {
    std::cout << "========================================\n";
    std::cout << "     Robotic Arm Kinematics Tool        \n";
    std::cout << "========================================\n\n";

    while (true) {
        std::cout << "Select Operation:\n";
        std::cout << "1. IK: Pose -> Joints\n";
        std::cout << "2. FK: Joints -> Pose\n";
        std::cout << "0. Exit\n";
        std::cout << "Choice: ";
        
        int choice;
        if (!(std::cin >> choice)) break;
        if (choice == 0) break;

        if (choice == 1) {
            Pose p;
            std::cout << "\nEnter Pose Parameters:\n";
            std::cout << "Yaw (deg): "; std::cin >> p.yaw;
            std::cout << "Pitch (deg): "; std::cin >> p.pitch;
            std::cout << "Z: "; std::cin >> p.a_z;
            std::cout << "Y: "; std::cin >> p.a_y;
            std::cout << "U5: "; std::cin >> p.u5;

            Joints j = inverse_kinematics(p);
            std::cout << "\n[RESULT] Joint Angles (theta 1-5):\n";
            std::cout << std::fixed << std::setprecision(6);
            std::cout << "T1: " << j.theta1 << "\n";
            std::cout << "T2: " << j.theta2 << "\n";
            std::cout << "T3: " << j.theta3 << "\n";
            std::cout << "T4: " << j.theta4 << "\n";
            std::cout << "T5: " << j.theta5 << "\n\n";
        } 
        else if (choice == 2) {
            Joints j;
            std::cout << "\nEnter Joint Angles (Theta 1-5):\n";
            std::cout << "T1: "; std::cin >> j.theta1;
            std::cout << "T2: "; std::cin >> j.theta2;
            std::cout << "T3: "; std::cin >> j.theta3;
            std::cout << "T4: "; std::cin >> j.theta4;
            std::cout << "T5: "; std::cin >> j.theta5;

            Pose p = forward_kinematics(j);
            std::cout << "\n[RESULT] Derived Pose:\n";
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "Yaw:   " << p.yaw << " deg\n";
            std::cout << "Pitch: " << p.pitch << " deg\n";
            std::cout << "Z:     " << p.a_z << "\n";
            std::cout << "Y:     " << p.a_y << "\n";
            std::cout << "U5:    " << p.u5 << "\n\n";
        }
        else {
            std::cout << "Invalid choice.\n";
        }
    }

    return 0;
}
