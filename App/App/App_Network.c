#include "main.h"
#include "lwip.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"

uint16_t counter = 0;
void send_data_tcp(void)
{
    int sock;
    struct sockaddr_in server_addr;
    char buf[64];

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) return;

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(5000);
    server_addr.sin_addr.s_addr = inet_addr("192.168.10.20");

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) == 0)
    {
		int len = sprintf(buf, "Hello from STM32H7 %u\r\n", counter++);
		send(sock, buf, len, 0);
        int len3 = sprintf(buf, "-----------------------\r\n");
        send(sock, buf, len3, 0);
    }

    closesocket(sock);
}
