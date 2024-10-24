#include <iostream>
#include <unistd.h>
#include <mavlink.h>
#include <arpa/inet.h>
#include <sys/socket.h>

constexpr auto TARGET_IP = "127.0.0.1";
constexpr auto TARGET_PORT = 18570;

float target_lat = 47.397742;
float target_lon = 8.545594;
float target_alt = 10; //yukseklik

int main() {

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);


    sockaddr_in sockaddr;
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(TARGET_PORT);
    inet_pton(AF_INET, TARGET_IP, &sockaddr.sin_addr);


    if (connect(sock, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
        std::cerr << "Bağlantı X" << std::endl;
        close(sock);
        return 1;
    }


    mavlink_message_t message;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];


    mavlink_command_long_t armCmd{};
    armCmd.target_system = 1;
    armCmd.target_component = 1;
    armCmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    armCmd.confirmation = 0;
    armCmd.param1 = 1.0;
    armCmd.param2 = 0;
    armCmd.param3 = 0;
    armCmd.param4 = 0;
    armCmd.param5 = 0;
    armCmd.param6 = 0;
    armCmd.param7 = 0;

    mavlink_msg_command_long_encode(1, 200, &message, &armCmd);
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);

    if (sendto(sock, buffer, len, 0, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
        std::cerr << "Arm X" << std::endl;
        close(sock);
        return 1;
    }

    std::cout << "ARM" << std::endl;


    mavlink_command_long_t takeoffCmd{};
    takeoffCmd.target_system = 1;
    takeoffCmd.target_component = 1;
    takeoffCmd.command = MAV_CMD_NAV_TAKEOFF;
    takeoffCmd.confirmation = 0;
    takeoffCmd.param1 = 0;
    takeoffCmd.param2 = 0;
    takeoffCmd.param3 = 0;
    takeoffCmd.param4 = 0;
    takeoffCmd.param5 = 0;
    takeoffCmd.param6 = 0;
    takeoffCmd.param7 = target_alt; // Yükseklik

    mavlink_msg_command_long_encode(1, 200, &message, &takeoffCmd);
    len = mavlink_msg_to_send_buffer(buffer, &message);

    if (sendto(sock, buffer, len, 0, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
        std::cerr << "Takeoff XX" << std::endl;
        close(sock);
        return 1;
    }

    std::cout << "Drone takeoff " << std::endl;


    mavlink_set_position_target_global_int_t gotoCmd{};
    gotoCmd.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    gotoCmd.type_mask = 0b0000111111111000;
    gotoCmd.lat_int = target_lat * 1e7;
    gotoCmd.lon_int = target_lon * 1e7;
    mavlink_msg_set_position_target_global_int_encode(1, 200, &message, &gotoCmd);
    len = mavlink_msg_to_send_buffer(buffer, &message);

    if (sendto(sock, buffer, len, 0, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
        std::cerr << "Hedef konuma gitme XXX." << std::endl;
        close(sock);
        return 1;
    }

    std::cout << "Hedef konuma gitme komutu iletildi" << std::endl;

    close(sock);
    return 0;
}

