#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <mavlink.h>

const char *udp_ip = "127.0.0.1";
const int udp_port = 18570;

int main() {
    // UDP socket oluşturma
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Socket oluşturulamadı." << std::endl;
        return -1;
    }

    // Drone adresi ve portu ayarlama
    struct sockaddr_in drone_addr;
    memset(&drone_addr, 0, sizeof(drone_addr));
    drone_addr.sin_family = AF_INET;
    drone_addr.sin_port = htons(udp_port);
    inet_pton(AF_INET, udp_ip, &drone_addr.sin_addr);

    // MAVLink mesajı ve komut oluşturma
    mavlink_message_t message;
    mavlink_status_t status;

    while (true) {
        // MAVLink mesajı al
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        ssize_t recsize = recv(sockfd, (void *)buf, MAVLINK_MAX_PACKET_LEN, 0);
        if (recsize > 0) {
            // Parse the message
            for (int i = 0; i < recsize; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &message, &status)) {
                    // Message received, process it
                    switch (message.msgid) {
                        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                            mavlink_global_position_int_t pos;
                            mavlink_msg_global_position_int_decode(&message, &pos);
                            std::cout << "Latitude: " << pos.lat / 1e7 << " deg, "
                                      << "Longitude: " << pos.lon / 1e7 << " deg, "
                                      << "Altitude: " << pos.alt / 1000.0 << " m" << std::endl;
                            break;
                        }
                        case MAVLINK_MSG_ID_SYS_STATUS: {
                            mavlink_sys_status_t sys_status;
                            mavlink_msg_sys_status_decode(&message, &sys_status);
                            std::cout << "Battery Voltage: " << sys_status.voltage_battery / 1000.0 << " V, "
                                      << "Battery Current: " << sys_status.current_battery / 100.0 << " A" << std::endl;
                            break;
                        }
                        // Add more message cases as needed
                    }
                }
            }
        }
        usleep(100000);  // Küçük bir bekleme süresi (0.1 saniye)
    }

    // Soketi kapat
    close(sockfd);

    return 0;
}









------------------------------------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------------------------------
#include <iostream>
#include <unistd.h>
#include <mavlink.h>
#include <arpa/inet.h>
#include <sys/socket.h>

constexpr auto TARGET_IP = "127.0.0.1";
constexpr auto TARGET_PORT = 18570;

int main() {
    // UDP socket oluşturma
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == -1) {
        std::cerr << "UDP socket oluşturulamadı." << std::endl;
        return 1;
    }

    // Bağlantı adresi tanımlama
    sockaddr_in sockaddr;
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(TARGET_PORT);
    inet_pton(AF_INET, TARGET_IP, &sockaddr.sin_addr);

    // Bağlantı kontrolü
    if (connect(sock, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
        std::cerr << "Bağlantı hatası." << std::endl;
        close(sock);
        return 1;
    }

    // Sürekli olarak GPS, hız ve yükseklik bilgisi alma döngüsü
    while (true) {
        // MAVLink mesajı ve buffer tanımlamaları
        mavlink_message_t message;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        // Veri alma işlemi
        ssize_t recsize = recv(sock, buffer, MAVLINK_MAX_PACKET_LEN, 0);
        if (recsize > 0) {
            // MAVLink mesajı işleme
            mavlink_status_t status;
            for (int i = 0; i < recsize; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status)) {
                    // GPS pozisyonu mesajını kontrol etme
                    if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                        mavlink_global_position_int_t gps;
                        mavlink_msg_global_position_int_decode(&message, &gps);
                        std::cout << "GPS: Lat " << gps.lat / 1e7 << ", Lon " << gps.lon / 1e7 << std::endl;
                    }
                    // Hız mesajını kontrol etme
                    else if (message.msgid == MAVLINK_MSG_ID_VFR_HUD) {
                        mavlink_vfr_hud_t vfr_hud;
                        mavlink_msg_vfr_hud_decode(&message, &vfr_hud);
                        std::cout << "Hız: " << vfr_hud.groundspeed << " m/s" << std::endl;
                    }
                    // Yükseklik mesajını kontrol etme
                    else if (message.msgid == MAVLINK_MSG_ID_ALTITUDE) {
                        mavlink_altitude_t altitude;
                        mavlink_msg_altitude_decode(&message, &altitude);
                        std::cout << "Yükseklik: " << altitude.altitude_amsl << " m" << std::endl;
                    }
                }
            }
        }

        // 1 saniye bekleme
        sleep(1);
    }

    close(sock);
    return 0;
}

