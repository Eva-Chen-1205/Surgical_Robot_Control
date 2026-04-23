#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <ctime>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <share.h>

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

// ============== IK Structs ==============
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
    double c_ori = std::sqrt((std::pow(L1_ori,2) + std::pow(h,2)) + std::pow(R,2) - 2 * std::sqrt(std::pow(L1_ori,2) + std::pow(h,2)) * R * cosd(theta_ori - alpha_ori));
    double cos_theta4_2_ori = (std::pow(A,2) + std::pow(c_ori,2) - std::pow(B,2)) / (2 * A * c_ori);
    double cos_theta4_1_ori = (std::pow(h,2) + std::pow(L1_ori,2) + std::pow(c_ori,2) - std::pow(R,2)) / (2 * std::sqrt(std::pow(h,2) + std::pow(L1_ori,2)) * c_ori);
    double theta4_1_ori = safe_acosd(cos_theta4_1_ori);
    double theta4_2_ori = safe_acosd(cos_theta4_2_ori);
    double theta4_ori = 270.0 - theta4_1_ori - theta4_2_ori - beta_ori;
    y.theta4 = (theta4 - theta4_ori) * PI / 180.0;

    return y;
}

// Wrapping Inverse Kinematics to respect the new origin Pose zero
Joints get_motor_angles(Pose p) {
    Pose home_pose = {-0.0031, 69.8409, -10.0357, -0.0045, 113.8079};
    Joints raw_home = inverse_kinematics(home_pose);
    Joints raw_p = inverse_kinematics(p);
    
    Joints corrected;
    corrected.theta1 = raw_p.theta1 - raw_home.theta1;
    corrected.theta2 = raw_p.theta2 - raw_home.theta2;
    corrected.theta3 = raw_p.theta3 - raw_home.theta3;
    corrected.theta4 = raw_p.theta4 - raw_home.theta4;
    corrected.theta5 = raw_p.theta5 - raw_home.theta5;
    
    // Snap tiny floating point artifacts exactly to 0
    if (std::abs(corrected.theta1) < 1e-9) corrected.theta1 = 0;
    if (std::abs(corrected.theta2) < 1e-9) corrected.theta2 = 0;
    if (std::abs(corrected.theta3) < 1e-9) corrected.theta3 = 0;
    if (std::abs(corrected.theta4) < 1e-9) corrected.theta4 = 0;
    if (std::abs(corrected.theta5) < 1e-9) corrected.theta5 = 0;

    return corrected;
}

// ============== SHA-1 Implementation ==============
static uint32_t sha1_rotl(uint32_t x, int n) { return (x << n) | (x >> (32 - n)); }

void sha1(const unsigned char* msg, size_t len, unsigned char hash[20]) {
    uint32_t sh0=0x67452301, sh1=0xEFCDAB89, sh2=0x98BADCFE, sh3=0x10325476, sh4=0xC3D2E1F0;
    uint64_t bit_len = (uint64_t)len * 8;
    
    // Padding
    size_t new_len = len + 1;
    while (new_len % 64 != 56) new_len++;
    unsigned char* padded = new unsigned char[new_len + 8];
    memcpy(padded, msg, len);
    padded[len] = 0x80;
    memset(padded + len + 1, 0, new_len - len - 1);
    for (int i = 0; i < 8; i++)
        padded[new_len + i] = (unsigned char)(bit_len >> (56 - 8*i));

    // Process blocks
    for (size_t offset = 0; offset < new_len + 8; offset += 64) {
        uint32_t w[80];
        for (int ii = 0; ii < 16; ii++)
            w[ii] = ((uint32_t)padded[offset+4*ii]<<24)|((uint32_t)padded[offset+4*ii+1]<<16)|((uint32_t)padded[offset+4*ii+2]<<8)|(uint32_t)padded[offset+4*ii+3];
        for (int ii = 16; ii < 80; ii++)
            w[ii] = sha1_rotl(w[ii-3]^w[ii-8]^w[ii-14]^w[ii-16], 1);

        uint32_t sa=sh0, sb=sh1, sc=sh2, sd=sh3, se=sh4;
        for (int ii = 0; ii < 80; ii++) {
            uint32_t sf, sk;
            if (ii < 20)      { sf=(sb&sc)|((~sb)&sd); sk=0x5A827999; }
            else if (ii < 40) { sf=sb^sc^sd;           sk=0x6ED9EBA1; }
            else if (ii < 60) { sf=(sb&sc)|(sb&sd)|(sc&sd); sk=0x8F1BBCDC; }
            else              { sf=sb^sc^sd;           sk=0xCA62C1D6; }
            uint32_t stemp = sha1_rotl(sa,5) + sf + se + sk + w[ii];
            se=sd; sd=sc; sc=sha1_rotl(sb,30); sb=sa; sa=stemp;
        }
        sh0+=sa; sh1+=sb; sh2+=sc; sh3+=sd; sh4+=se;
    }
    delete[] padded;

    uint32_t hh[5] = {sh0,sh1,sh2,sh3,sh4};
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 4; j++)
            hash[i*4+j] = (unsigned char)(hh[i] >> (24-8*j));
}

// ============== Base64 Encode ==============
std::string base64_encode(const unsigned char* data, size_t len) {
    static const char* chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string result;
    int val = 0, valb = -6;
    for (size_t i = 0; i < len; i++) {
        val = (val << 8) + data[i];
        valb += 8;
        while (valb >= 0) {
            result.push_back(chars[(val >> valb) & 0x3F]);
            valb -= 6;
        }
    }
    if (valb > -6) result.push_back(chars[((val << 8) >> (valb + 8)) & 0x3F]);
    while (result.size() % 4) result.push_back('=');
    return result;
}

// ============== WebSocket Helpers ==============
bool ws_handshake(SOCKET client) {
    char buf[4096] = {0};
    int n = recv(client, buf, sizeof(buf)-1, 0);
    if (n <= 0) return false;
    
    std::string req(buf, n);
    // Find Sec-WebSocket-Key
    std::string key_header = "Sec-WebSocket-Key: ";
    size_t pos = req.find(key_header);
    if (pos == std::string::npos) return false;
    
    size_t start = pos + key_header.size();
    size_t end = req.find("\r\n", start);
    std::string key = req.substr(start, end - start);
    
    // Concatenate with magic GUID
    key += "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    
    // SHA-1 hash
    unsigned char hash[20];
    sha1((const unsigned char*)key.c_str(), key.size(), hash);
    
    // Base64 encode
    std::string accept = base64_encode(hash, 20);
    
    // Send upgrade response
    std::string response = 
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " + accept + "\r\n\r\n";
    
    send(client, response.c_str(), (int)response.size(), 0);
    return true;
}

// Read one WebSocket frame, returns payload string. Empty string on error/close.
std::string ws_read_frame(SOCKET client) {
    auto recv_all = [&](unsigned char* buf, int len) {
        int r = 0;
        while (r < len) {
            int chunk = recv(client, (char*)(buf + r), len - r, 0);
            if (chunk <= 0) return false;
            r += chunk;
        }
        return true;
    };

    std::string full_payload = "";
    bool fin = false;

    while (!fin) {
        unsigned char header[2];
        if (!recv_all(header, 2)) return "";
        
        fin = (header[0] & 0x80) != 0;
        int opcode = header[0] & 0x0F;
        if (opcode == 0x8) return ""; // Close frame
        
        bool masked = (header[1] & 0x80) != 0;
        uint64_t payload_len = header[1] & 0x7F;
        
        if (payload_len == 126) {
            unsigned char ext[2];
            if (!recv_all(ext, 2)) return "";
            payload_len = (ext[0] << 8) | ext[1];
        } else if (payload_len == 127) {
            unsigned char ext[8];
            if (!recv_all(ext, 8)) return "";
            payload_len = 0;
            for (int i = 0; i < 8; i++)
                payload_len = (payload_len << 8) | ext[i];
        }
        
        unsigned char mask[4] = {0};
        if (masked && !recv_all(mask, 4)) return "";
        
        if (payload_len > 0) {
            std::string payload(payload_len, '\0');
            uint64_t received = 0;
            while (received < payload_len) {
                int r = recv(client, &payload[received], (int)(payload_len - received), 0);
                if (r <= 0) return "";
                received += r;
            }
            
            if (masked) {
                for (uint64_t i = 0; i < payload_len; i++)
                    payload[i] ^= mask[i % 4];
            }
            full_payload += payload;
        }
    }
    
    return full_payload;
}

// Send a WebSocket text frame
void ws_send_frame(SOCKET client, const std::string& msg) {
    std::vector<unsigned char> frame;
    frame.push_back(0x81); // FIN + text opcode
    
    if (msg.size() <= 125) {
        frame.push_back((unsigned char)msg.size());
    } else if (msg.size() <= 65535) {
        frame.push_back(126);
        frame.push_back((unsigned char)(msg.size() >> 8));
        frame.push_back((unsigned char)(msg.size() & 0xFF));
    } else {
        frame.push_back(127);
        for (int i = 7; i >= 0; i--)
            frame.push_back((unsigned char)(msg.size() >> (8*i)));
    }
    
    for (char c : msg) frame.push_back((unsigned char)c);
    send(client, (const char*)frame.data(), (int)frame.size(), 0);
}

// ============== JSON Array Parser ==============
std::vector<double> parse_json_array(const std::string& body) {
    std::vector<double> res;
    size_t start = body.find('[');
    size_t end = body.find(']');
    if (start != std::string::npos && end != std::string::npos && end > start) {
        std::string inner = body.substr(start + 1, end - start - 1);
        std::stringstream ss(inner);
        std::string token;
        while (std::getline(ss, token, ',')) {
            res.push_back(std::stod(token));
        }
    }
    return res;
}

// ============== Main ==============
int main() {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed.\n";
        return 1;
    }

    SOCKET server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(8080);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) == SOCKET_ERROR) {
        std::cerr << "Bind failed.\n";
        WSACleanup();
        return 1;
    }
    listen(server_fd, 3);
    // Generate timestamp filename
    std::time_t now = std::time(nullptr);
    std::tm* lt = std::localtime(&now);
    char fname[128];
    std::strftime(fname, sizeof(fname), "ik_trajectory_%Y%m%d_%H%M%S.txt", lt);
    std::string trajectory_file(fname);

    std::cout << "=== IK WebSocket Server ===\n";
    std::cout << "Trajectory file: " << trajectory_file << "\n";
    std::cout << "Listening on port 8080...\n";
    std::cout << "Waiting for browser connection...\n\n";

    // Use _fsopen to allow shared read access by fk_test.exe
    FILE* shared_file = _fsopen(trajectory_file.c_str(), "w", _SH_DENYNO);
    if (!shared_file) {
        std::cerr << "Failed to open trajectory file for writing.\n";
        WSACleanup();
        return 1;
    }

    while (true) {
        SOCKET client_socket = accept(server_fd, nullptr, nullptr);
        if (client_socket == INVALID_SOCKET) continue;

        // Perform WebSocket handshake
        if (!ws_handshake(client_socket)) {
            std::cerr << "WebSocket handshake failed.\n";
            closesocket(client_socket);
            continue;
        }

        std::cout << "[Connected] Browser client connected!\n\n";

        // Persistent connection loop - keep reading frames
        while (true) {
            std::string payload = ws_read_frame(client_socket);
            if (payload.empty()) {
                std::cout << "\n[Disconnected] Client disconnected.\n";
                std::cout << "Waiting for new connection...\n\n";
                break;
            }

            // Direct file save command
            if (payload.rfind("SAVE:", 0) == 0) {
                size_t delim = payload.find('|', 5);
                if (delim != std::string::npos) {
                    std::string filename = payload.substr(5, delim - 5);
                    std::string content = payload.substr(delim + 1);
                    std::ofstream out(filename, std::ios::out | std::ios::binary);
                    if (out) {
                        out << content;
                        out.close();
                        std::cout << "\n[SAVE] Exported trajectory to " << filename << "\n";
                        ws_send_frame(client_socket, "{\"status\":\"success\", \"file\":\"" + filename + "\"}");
                    } else {
                        ws_send_frame(client_socket, "{\"status\":\"error\", \"error\":\"File write failed\"}");
                    }
                }
                continue;
            }

            std::vector<double> u = parse_json_array(payload);
            if (u.size() == 5) {
                Pose p = {u[0], u[1], u[2], u[3], u[4]};
                Joints j = get_motor_angles(p);

                // Real-time console output
                std::cout << "\r                                                                              ";
                std::cout << "\rInput [Yaw=" << p.yaw << " Pitch=" << p.pitch 
                          << " Z=" << p.a_z << " Y=" << p.a_y << " U5=" << p.u5 << "]"
                          << " -> Theta[" << j.theta1 << ", " << j.theta2 << ", " 
                          << j.theta3 << ", " << j.theta4 << ", " << j.theta5 << "]" << std::flush;

                // Save to file
                // Use fprintf instead of outfile << for the FILE* handle
                fprintf(shared_file, "%.6f, %.6f, %.6f, %.6f, %.6f\n", 
                        j.theta1, j.theta2, j.theta3, j.theta4, j.theta5);
                fflush(shared_file);

                // Send result back to browser
                std::ostringstream oss;
                oss << "{\"theta1\":" << j.theta1 
                    << ",\"theta2\":" << j.theta2 
                    << ",\"theta3\":" << j.theta3 
                    << ",\"theta4\":" << j.theta4 
                    << ",\"theta5\":" << j.theta5 << "}";
                ws_send_frame(client_socket, oss.str());
            }
        }
        closesocket(client_socket);
    }

    WSACleanup();
    return 0;
}
