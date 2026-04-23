#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <string>
#include <windows.h>

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

    double x = 25;
    double h1 = DD_1 - 30;
    double d_total = p.u5 - 74.2;
    double d_2 = x * cosd(90.0 - Pitch) + (x * sind(90.0 - Pitch) + h1) * cosd(Pitch)/sind(Pitch);
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
    double theta5 = h1 / cosd(90.0 - Pitch) + x * tand(90.0 - Pitch);
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
    // Fixed: use L1_ori (not L1) for the original configuration calculation
    double c_ori = std::sqrt((std::pow(L1_ori,2) + std::pow(h,2)) + std::pow(R,2) - 2 * std::sqrt(std::pow(L1_ori,2) + std::pow(h,2)) * R * cosd(theta_ori - alpha_ori));
    double cos_theta4_2_ori = (std::pow(A,2) + std::pow(c_ori,2) - std::pow(B,2)) / (2 * A * c_ori);
    double cos_theta4_1_ori = (std::pow(h,2) + std::pow(L1_ori,2) + std::pow(c_ori,2) - std::pow(R,2)) / (2 * std::sqrt(std::pow(h,2) + std::pow(L1_ori,2)) * c_ori);
    double theta4_1_ori = safe_acosd(cos_theta4_1_ori);
    double theta4_2_ori = safe_acosd(cos_theta4_2_ori);
    double theta4_ori = 270.0 - theta4_1_ori - theta4_2_ori - beta_ori;
    y.theta4 = (theta4 - theta4_ori) * PI / 180.0;

    return y;
}

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
        for (int k = i - 1; k >= 0; k--) {
            b[k] -= A[k][i] * x[i];
        }
    }
    return x;
}

bool has_nan(const std::vector<double>& v) {
    for (auto x : v) if (std::isnan(x) || std::isinf(x)) return true;
    return false;
}

double vec_norm_sq(const std::vector<double>& v) {
    double s = 0;
    for (auto x : v) s += x * x;
    return s;
}

Pose forward_kinematics_solver(const Joints& target, Pose initial_guess) {
    std::vector<double> tgt = { target.theta1, target.theta2, target.theta3, target.theta4, target.theta5 };
    
    // Try multiple initial guesses if the first one fails
    std::vector<std::vector<double>> guesses = {
        { initial_guess.yaw, initial_guess.pitch, initial_guess.a_z, initial_guess.a_y, initial_guess.u5 },
        { -0.0031, 69.8409, -10.0357, -0.0045, 113.8079 },
        { 0, 65, -10, 0, 110 },
        { 0, 75, -10, 0, 120 },
        { 5, 70, -12, 0, 115 },
        { -5, 70, -8, 0, 115 },
    };
    
    Pose best_result = initial_guess;
    double best_err = 1e30;
    
    for (auto& guess : guesses) {
        std::vector<double> u = guess;
        double lambda = 1e-3;  // Levenberg-Marquardt damping factor
        bool converged = false;
        
        for (int iter = 0; iter < 500; iter++) {
            Pose p = {u[0], u[1], u[2], u[3], u[4]};
            Joints y = inverse_kinematics(p);
            std::vector<double> current_y = { y.theta1, y.theta2, y.theta3, y.theta4, y.theta5 };
            
            // Check for NaN in IK output
            if (has_nan(current_y)) {
                lambda *= 10;
                if (lambda > 1e6) break;
                continue;
            }
            
            std::vector<double> err(5);
            double max_err = 0;
            for(int i=0; i<5; i++) {
                err[i] = current_y[i] - tgt[i];
                if (std::abs(err[i]) > max_err) max_err = std::abs(err[i]);
            }
            
            if (max_err < 1e-8) {
                converged = true;
                best_result = {u[0], u[1], u[2], u[3], u[4]};
                best_err = max_err;
                break;
            }

            // Compute Jacobian using central differences for better accuracy
            std::vector<std::vector<double>> J(5, std::vector<double>(5));
            double delta = 1e-6;
            for (int j = 0; j < 5; j++) {
                std::vector<double> u_plus = u, u_minus = u;
                u_plus[j] += delta;
                u_minus[j] -= delta;
                Pose p_plus = {u_plus[0], u_plus[1], u_plus[2], u_plus[3], u_plus[4]};
                Pose p_minus = {u_minus[0], u_minus[1], u_minus[2], u_minus[3], u_minus[4]};
                Joints y_plus = inverse_kinematics(p_plus);
                Joints y_minus = inverse_kinematics(p_minus);
                std::vector<double> yp = { y_plus.theta1, y_plus.theta2, y_plus.theta3, y_plus.theta4, y_plus.theta5 };
                std::vector<double> ym = { y_minus.theta1, y_minus.theta2, y_minus.theta3, y_minus.theta4, y_minus.theta5 };
                
                // If central diff produces NaN, fall back to forward diff
                for(int i=0; i<5; i++) {
                    double jval = (yp[i] - ym[i]) / (2 * delta);
                    if (std::isnan(jval) || std::isinf(jval)) {
                        jval = (yp[i] - current_y[i]) / delta;
                    }
                    if (std::isnan(jval) || std::isinf(jval)) jval = 0;
                    J[i][j] = jval;
                }
            }

            // Levenberg-Marquardt: solve (J^T * J + lambda * I) * du = -J^T * err
            // Compute J^T * J and J^T * err
            std::vector<std::vector<double>> JTJ(5, std::vector<double>(5, 0));
            std::vector<double> JTe(5, 0);
            for(int i=0; i<5; i++) {
                for(int j=0; j<5; j++) {
                    for(int k=0; k<5; k++) {
                        JTJ[i][j] += J[k][i] * J[k][j];
                    }
                    JTe[i] += J[j][i] * err[j];  // J^T * err (not neg_err)
                }
            }
            
            // Add damping: JTJ + lambda * diag(JTJ)
            for(int i=0; i<5; i++) {
                double diag_val = JTJ[i][i];
                if (diag_val < 1e-10) diag_val = 1e-10;
                JTJ[i][i] += lambda * diag_val;
            }
            
            // Negate JTe for solving
            for(int i=0; i<5; i++) JTe[i] = -JTe[i];
            
            std::vector<double> du = solve_linear_system(JTJ, JTe);
            
            // Check for NaN in Newton step
            if (has_nan(du)) {
                lambda *= 10;
                if (lambda > 1e6) break;
                continue;
            }
            
            // Line search: try full step, then halve
            double old_err_norm = vec_norm_sq(err);
            bool step_accepted = false;
            double step_size = 1.0;
            
            for (int ls = 0; ls < 10; ls++) {
                std::vector<double> u_trial(5);
                for(int i=0; i<5; i++) u_trial[i] = u[i] + step_size * du[i];
                
                Pose p_trial = {u_trial[0], u_trial[1], u_trial[2], u_trial[3], u_trial[4]};
                Joints y_trial = inverse_kinematics(p_trial);
                std::vector<double> y_trial_vec = { y_trial.theta1, y_trial.theta2, y_trial.theta3, y_trial.theta4, y_trial.theta5 };
                
                if (has_nan(y_trial_vec)) {
                    step_size *= 0.5;
                    continue;
                }
                
                std::vector<double> new_err(5);
                for(int i=0; i<5; i++) new_err[i] = y_trial_vec[i] - tgt[i];
                double new_err_norm = vec_norm_sq(new_err);
                
                if (new_err_norm < old_err_norm || ls == 9) {
                    u = u_trial;
                    step_accepted = true;
                    if (new_err_norm < old_err_norm) {
                        lambda = std::max(lambda * 0.5, 1e-10);
                    } else {
                        lambda *= 5;
                    }
                    break;
                }
                step_size *= 0.5;
            }
            
            if (!step_accepted) {
                lambda *= 10;
                if (lambda > 1e6) break;
            }
        }
        
        if (converged) break;
        
        // Check if this guess gave a better result than previous
        Pose p_final = {u[0], u[1], u[2], u[3], u[4]};
        Joints y_final = inverse_kinematics(p_final);
        std::vector<double> final_y = { y_final.theta1, y_final.theta2, y_final.theta3, y_final.theta4, y_final.theta5 };
        if (!has_nan(final_y)) {
            double final_err = 0;
            for(int i=0; i<5; i++) final_err = std::max(final_err, std::abs(final_y[i] - tgt[i]));
            if (final_err < best_err) {
                best_err = final_err;
                best_result = {u[0], u[1], u[2], u[3], u[4]};
            }
        }
    }
    
    return best_result;
}

// Find the latest ik_trajectory_*.txt file in the current directory
std::string find_latest_ik_trajectory() {
    WIN32_FIND_DATAA fdata;
    HANDLE hFind = FindFirstFileA("ik_trajectory_*.txt", &fdata);
    if (hFind == INVALID_HANDLE_VALUE) return "";
    
    std::string latest = fdata.cFileName;
    while (FindNextFileA(hFind, &fdata)) {
        std::string name = fdata.cFileName;
        if (name > latest) latest = name;  // lexicographic comparison works for YYYYMMDD_HHMMSS
    }
    FindClose(hFind);
    return latest;
}

int main(int argc, char* argv[]) {
    // Determine input file
    std::string input_file;
    if (argc > 1) {
        input_file = argv[1];
    } else {
        // Auto-find latest ik_trajectory file
        input_file = find_latest_ik_trajectory();
        if (input_file.empty()) {
            // Fallback to old name
            input_file = "ik_trajectory.txt";
        }
    }

    std::ifstream infile(input_file);
    if (!infile.is_open()) {
        std::cerr << "Failed to open " << input_file << "\n";
        std::cerr << "Usage: fk_test [ik_trajectory_file.txt]\n";
        return 1;
    }
    
    std::string output_file = "fk_" + input_file;
    std::ofstream outfile(output_file);

    std::cout << "=== FK Verification Tool ===\n";
    std::cout << "Input:  " << input_file << "\n";
    std::cout << "Output: " << output_file << "\n";
    std::cout << "----------------------------\n";

    std::string line;
    Pose last_pose = {-0.0031, 69.8409, -10.0357, -0.0045, 113.8079}; // Base initial guess
    int line_num = 0;

    while (true) {
        if (std::getline(infile, line)) {
            if (line.empty()) continue;
            // Strip \r if present
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;
            
            std::stringstream ss(line);
            std::string token;
            std::vector<double> thetas;
            while (std::getline(ss, token, ',')) {
                thetas.push_back(std::stod(token));
            }

            if (thetas.size() >= 5) {
                line_num++;
                Joints tgt = {thetas[0], thetas[1], thetas[2], thetas[3], thetas[4]};
                Pose fk_pose = forward_kinematics_solver(tgt, last_pose);
                last_pose = fk_pose;
                
                outfile << std::fixed << std::setprecision(4)
                        << fk_pose.yaw << ", " 
                        << fk_pose.pitch << ", " 
                        << fk_pose.a_z << ", " 
                        << fk_pose.a_y << ", " 
                        << fk_pose.u5 << "\n";
                // Flush so output is immediate
                outfile.flush();
                
                std::cout << "[" << std::setw(3) << line_num << "] "
                          << "IK theta: [" << std::fixed << std::setprecision(4)
                          << thetas[0] << ", " << thetas[1] << ", " << thetas[2] << ", " 
                          << thetas[3] << ", " << thetas[4] << "]"
                          << " -> FK pose: [Yaw=" << fk_pose.yaw 
                          << " Pitch=" << fk_pose.pitch 
                          << " Z=" << fk_pose.a_z 
                          << " Y=" << fk_pose.a_y 
                          << " U5=" << fk_pose.u5 << "]\n";
            }
        } else {
            // Clear EOF flag and wait for more data to be written
            infile.clear();
            Sleep(50);
        }
    }
    std::cout << "----------------------------\n";
    std::cout << "Processed " << line_num << " lines. Results saved to " << output_file << "\n";
    return 0;
}
