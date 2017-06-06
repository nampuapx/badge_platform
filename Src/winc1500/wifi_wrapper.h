
//#define WIFIWINC1500
//#define WIFICC3000

#ifndef WIFI_WRAPPER_DEF_

#define WIFI_WRAPPER_DEF_



//#include "leds.h"

#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"

void WINC_SPI_callback(uint32_t event);

void WINC1500_setup(void);

//void WINC_test_task(void);
typedef struct
{
	char ssid[20];
	char pass[20];
} WifiNetworkListItem;

void WINC_WIFI_new_connect_unblocking(char* ssid,char* pass);
uint8_t WINC_WIFI_new_connect(char* ssid,char* pass,uint32_t timeout_millisec);
uint8_t WINC_WIFI_def_connect(uint32_t timeout_millisec);
uint8_t WINC_WIFI_is_connected(void);
void WINC_WIFI_reset_init(void);
void WINC_http_socket_send(void *pvSendBuffer, uint16_t u16SendLength);
void WINC_http_socket_send_unblocking(void *pvSendBuffer, uint16_t u16SendLength);
int WINC_http_socket_open_by_ip(uint32_t ip, uint16_t port);
void WINC_http_socket_open(char* host_name, uint16_t port);
void WINC_http_socket_close(int socket);
int16_t WINC_http_socket_recv(void *pvRecvBuf, uint16 u16BufLen, uint32 u32Timeoutmsec);
void WINC_http_socket_recv_unblocking(void *pvRecvBuf, uint16 u16BufLen, uint32 u32Timeoutmsec);				
uint16_t WINC_http_socket_recv_block_unblocking(void);	
uint8_t WINC_WIFI_new_connect_with_scan_list(WifiNetworkListItem* network_list, uint8_t network_list_count);

#define MAIN_WLAN_SSID        "FlexiBadge" /* < Destination SSID */
#define MAIN_WLAN_AUTH        M2M_WIFI_SEC_WPA_PSK /* < Security manner */
#define MAIN_WLAN_PSK         "1234567890" /* < Password for Destination SSID */

#define WIFI_CONNECT_TIMEOUT	20000


enum {
	SEND_DONE_SIG = 0x0001,
	CONNECT_DONE_SIG = 0x0002,
	RECV_DONE_SIG = 0x0004,
	WINC_ISR = 0x0008,
	WINC_WIFI_CONNECT_DONE = 0x0010
};

typedef enum{
	TCP_DONE,
	TCP_FAILED
}_TCPconnRESULT;



extern _TCPconnRESULT TCPconnRESULT;


extern osThreadId WINK_ISR_H_task_Thread_ID;

#define DEV_NAME "FlixiBadge"

void WINC1500_loop(void);

void isr(void);
#endif




//int send_wrap(long sd, const void *buf, long len, long flags);
//long connect_socket_wrap(long sd, const void *addr, long addrlen);
//extern int recv_wrap(long sd, void *buf, long len, long flags);
//extern int socket_wrap(long domain, long type, long protocol);
//////long connect_socket_wrap(long sd, const sockaddr *addr, long addrlen);
//long closesocket_wrap(long sd);


