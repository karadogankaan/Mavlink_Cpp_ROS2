#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <mavlink.h>

constexpr auto TARGET_IP = "127.0.0.1";
constexpr auto TARGET_PORT = 18570;

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
    armCmd.param1 = 1.0; // Arm=1 disarm=0 
    armCmd.param2 = 0;
    armCmd.param3 = 0;
    armCmd.param4 = 0;
    armCmd.param5 = 0;
    armCmd.param6 = 0;
    armCmd.param7 = 0;

    mavlink_msg_command_long_encode(1, 200, &message, &armCmd);
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);

    if (sendto(sock, buffer, len, 0, reinterpret_cast<sockaddr*>(&targetAddr), sizeof(targetAddr)) < 0) {
        std::cerr << "Arm XXX." << std::endl;
        close(sock);
        return -1;
    }

    std::cout << "Arm komut ->." << std::endl;

    sleep(5);
    close(sock);
    return 0;
}


