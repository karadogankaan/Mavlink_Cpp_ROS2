
#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <mavlink.h>

constexpr auto TARGET_IP = "127.0.0.1";
constexpr auto TARGET_PORT = 18570;

void send_heartbeat(int sock, sockaddr_in& targetAddr) {
    mavlink_message_t message;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_heartbeat_t heartbeat{};
    heartbeat.type = MAV_TYPE_GENERIC;
    heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
    heartbeat.base_mode = 0;
    heartbeat.custom_mode = 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;

    mavlink_msg_heartbeat_encode(1, 200, &message, &heartbeat);
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);

    sendto(sock, buffer, len, 0, reinterpret_cast<sockaddr*>(&targetAddr), sizeof(targetAddr));
}

int main(int argc, char *argv[]) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    
    sockaddr_in targetAddr{};
    targetAddr.sin_family = AF_INET;
    targetAddr.sin_port = htons(TARGET_PORT);
    inet_pton(AF_INET, TARGET_IP, &targetAddr.sin_addr);

    mavlink_message_t message;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    
    mavlink_command_long_t armCmd{};
    armCmd.target_system = 1;
    armCmd.target_component = 1;
    armCmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    armCmd.confirmation = 0;
    armCmd.param1 = 1.0; // Arm
    armCmd.param2 = 0;
    armCmd.param3 = 0;
    armCmd.param4 = 0;
    armCmd.param5 = 0;
    armCmd.param6 = 0;
    armCmd.param7 = 0;

    mavlink_msg_command_long_encode(1, 200, &message, &armCmd);
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);

    if (sendto(sock, buffer, len, 0, reinterpret_cast<sockaddr*>(&targetAddr), sizeof(targetAddr)) < 0) {
        std::cerr << "Arm komutu X" << std::endl;
        close(sock);
        return -1;
    }

    std::cout << "Arm komutu Gonderildi" << std::endl;

    for (int i = 0; i < 5; ++i) {
        send_heartbeat(sock, targetAddr);
        sleep(1);
    }

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
    takeoffCmd.param7 = 10.0; //yukseklik

    mavlink_msg_command_long_encode(1, 200, &message, &takeoffCmd);
    len = mavlink_msg_to_send_buffer(buffer, &message);

    if (sendto(sock, buffer, len, 0, reinterpret_cast<sockaddr*>(&targetAddr), sizeof(targetAddr)) < 0) {
        std::cerr << "Takeoff komutu X." << std::endl;
        close(sock);
        return -1;
    }

    std::cout << "Takeoff komutu gGonderildi." << std::endl;


    while (true) {
        send_heartbeat(sock, targetAddr);
        std::cout <<"heartbeat devam ediyor." << std::endl;
        sleep(1);
    }

    close(sock);
    return 0;
}
