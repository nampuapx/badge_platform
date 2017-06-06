
#include <stdio.h>
#include <string.h>

#include "wifi_wrapper.h"
#include "socket/include/socket.h"
#include "cmsis_os.h"                   // CMSIS RTOS header file

////////////////////////////////////////////////////////////////
static uint8_t scan_request_index = 0;
char date[] = {__DATE__};
static uint8_t num_founded_ap = 0;

char errout[100];

uint8_t WINC_busy = 0;

char _ssid[64];
char _pass[64];

WifiNetworkListItem* wifi_network_allowed_list = NULL;
uint8_t wifi_network_allowed_list_count = 0;

uint8_t WINC_WIFI_is_conn = 0;
osThreadId WINC_WIFI_Thread_ID;

SOCKET clientSocket_FTP_HTTP_narmon;
SOCKET clientSocket_FTP_HTTP_1;
osThreadId clientSocket_FTP_HTTP_1_Thread_ID;
uint16_t clientSocket_FTP_HTTP_1_port;

int16_t clientSocket_FTP_HTTP_1_recv_umt;

_TCPconnRESULT TCPconnRESULT;

	 char host_name[] = {"narodmon.ru"};
	 
	 
	 char err;
////////////////////////////////////////////////////////////////////

void WINK_ISR_H_task(void const *p);
osThreadDef(WINK_ISR_H_task, osPriorityNormal, 1, 0);
osThreadId WINK_ISR_H_task_Thread_ID;

osMutexDef (winc_spi_mutex);    // Declare mutex
osMutexId  (winc_spi_mutex_id); // Mutex ID



/* Socket event handler.
*/
void tcpClientSocketEventHandler(SOCKET sock, uint8 u8Msg, void * pvMsg)
{
			tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg*)pvMsg;
	
	if(sock == clientSocket_FTP_HTTP_1){
		switch(u8Msg){
			case SOCKET_MSG_CONNECT:
				if(pstrConnect->s8Error == 0){
						TCPconnRESULT = TCP_DONE;
						osSignalSet(clientSocket_FTP_HTTP_1_Thread_ID, CONNECT_DONE_SIG);
					}else{
							TCPconnRESULT = TCP_FAILED;
							osSignalSet(clientSocket_FTP_HTTP_1_Thread_ID, CONNECT_DONE_SIG);
						//printf("TCP Connection Failed\n");
						LED_on(LED_RED);
						//__breakpoint(0);
					}
				break;
				case SOCKET_MSG_RECV:
				tstrSocketRecvMsg *pstrRecvMsg = (tstrSocketRecvMsg*)pvMsg;
				if((pstrRecvMsg->pu8Buffer != NULL) && (pstrRecvMsg->s16BufferSize > 0)){
					clientSocket_FTP_HTTP_1_recv_umt = pstrRecvMsg->s16BufferSize;
					
				}
				osSignalSet(clientSocket_FTP_HTTP_1_Thread_ID, RECV_DONE_SIG);
				break;
				case SOCKET_MSG_SEND:
					osSignalSet(clientSocket_FTP_HTTP_1_Thread_ID, SEND_DONE_SIG);
				break;				
				default:
					LED_on(LED_BLUE);
				break;
		}

	}//else if(sock == clientSocket_FTP_HTTP_narmon)
//	{
//		//if(u8Msg == SOCKET_MSG_CONNECT)
//		switch(u8Msg){
//			case SOCKET_MSG_CONNECT:
//		// Connect Event Handler.

//			if(pstrConnect->s8Error == 0)
//			{
//			// Perform data exchange.
//			uint8 acSendBuffer[256] = {"#F8-F0-05-F0-1A-F7#TEN_Electronics\n#U1#1.23\n##"};
//			uint16 u16MsgSize;
//			u16MsgSize = strlen(acSendBuffer);
//		// Fill in the acSendBuffer with some data here
//		// send data
//			send(clientSocket_FTP_HTTP_narmon, acSendBuffer, u16MsgSize, 0);
//			// Recv response from server.
//			recv(clientSocket_FTP_HTTP_narmon, rxBuffer, sizeof(rxBuffer), 0);
//			}
//			else
//			{
//				//printf("TCP Connection Failed\n");
//				__breakpoint(0);
//			}
//			break;
//		//else if(u8Msg == SOCKET_MSG_RECV)
//			case SOCKET_MSG_RECV:
//			tstrSocketRecvMsg *pstrRecvMsg = (tstrSocketRecvMsg*)pvMsg;
//			if((pstrRecvMsg->pu8Buffer != NULL) && (pstrRecvMsg->s16BufferSize > 0))
//			{
//// Process the received message.
//// Close the socket.
//				close(clientSocket_FTP_HTTP_narmon);
//				__nop();
//			}
//			break;
//			case SOCKET_MSG_SEND:
//				__nop();
//			break;
//		}
//	}
}
// This is the DNS callback. The response of gethostbyname is here.
void dnsResolveCallback(uint8* pu8HostName, uint32 u32ServerIP)
{
	struct sockaddr_in strAddr;
	
	if(u32ServerIP != 0)
		{
			clientSocket_FTP_HTTP_1 = socket(AF_INET,SOCK_STREAM,0);
			if(clientSocket_FTP_HTTP_1 >= 0){
				strAddr.sin_family = AF_INET;
				strAddr.sin_port = _htons(clientSocket_FTP_HTTP_1_port);
				strAddr.sin_addr.s_addr = u32ServerIP;
				connect(clientSocket_FTP_HTTP_1, (struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));
			}
		}else{
			//printf("DNS Resolution Failed\n");
			__breakpoint(0);
		}	
	
//	if(u32ServerIP != 0)
//		{
//			clientSocket_FTP_HTTP_narmon = socket(AF_INET,SOCK_STREAM,0);
//			if(clientSocket_FTP_HTTP_narmon >= 0){
//				strAddr.sin_family = AF_INET;
//				strAddr.sin_port = _htons(8283);
//				strAddr.sin_addr.s_addr = u32ServerIP;
//				connect(clientSocket_FTP_HTTP_narmon, (struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));
//			}
//		}else{
//			//printf("DNS Resolution Failed\n");
//			__nop();
//		}
}



//	struct sockaddr_in strAddr;		
//			clientSocketHdl = socket(AF_INET,SOCK_STREAM,0);
//			if(clientSocketHdl >= 0){
//				strAddr.sin_family = AF_INET;
//				strAddr.sin_port = _htons(8283);
//				strAddr.sin_addr.s_addr = nmi_inet_addr("94.142.140.101");
//				connect(clientSocketHdl, (struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));
//			}






/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_DHCP_CONF](@ref M2M_WIFI_REQ_DHCP_CONF)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type.
 */
 static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_SCAN_DONE:
	{
		tstrM2mScanDone *pstrInfo = (tstrM2mScanDone *)pvMsg;
		num_founded_ap = m2m_wifi_get_num_ap_found();
		scan_request_index = 0;
		
		//if (pstrInfo->u8NumofCh >= 1) {
		if (num_founded_ap >= 1)
		{
			m2m_wifi_req_scan_result(scan_request_index);
			scan_request_index++;
		} 
		else 
		{
			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_RESP_SCAN_RESULT:
	{
		tstrM2mWifiscanResult *pstrScanResult = (tstrM2mWifiscanResult *)pvMsg;
		uint16_t demo_ssid_len;
		uint16_t scan_ssid_len = strlen((const char *)pstrScanResult->au8SSID);
		char* scanned_ssid = pstrScanResult->au8SSID;
		
		
		//Check the acquired ssid with ssid from our list
		for (uint8_t i=0; i<wifi_network_allowed_list_count; i++)
		{
			//If we got ssid that present in list
			if (strcmp(scanned_ssid, wifi_network_allowed_list[i].ssid) == 0)
			{
				m2m_wifi_connect((char *)scanned_ssid,
						//sizeof(MAIN_WLAN_SSID),
						strlen(scanned_ssid),
						MAIN_WLAN_AUTH,
						//(void *)MAIN_WLAN_PSK,
						wifi_network_allowed_list[i].pass,
						M2M_WIFI_CH_ALL);
				break;
			}
		}

		/* display founded AP. */
		//printf("[%d] SSID:%s\r\n", scan_request_index, pstrScanResult->au8SSID);

		//num_founded_ap = m2m_wifi_get_num_ap_found();
//		if (scan_ssid_len) {
//			/* check same SSID. */
//			demo_ssid_len = strlen((const char *)_ssid);
//			if
//			(
//				(demo_ssid_len == scan_ssid_len) &&
//				(!memcmp(pstrScanResult->au8SSID, (uint8_t *)_ssid, demo_ssid_len))
//			) {
//				/* A scan result matches an entry in the preferred AP List.
//				 * Initiate a connection request.
//				 */
//				//printf("Found %s \r\n", MAIN_WLAN_SSID);
//						__nop();

//				m2m_wifi_connect((char *)_ssid,
//						//sizeof(MAIN_WLAN_SSID),
//						strlen(_ssid),
//						MAIN_WLAN_AUTH,
//						//(void *)MAIN_WLAN_PSK,
//						_pass,
//						M2M_WIFI_CH_ALL);
//				break;
//			}
//		}

		if (scan_request_index < num_founded_ap) 
		{
			m2m_wifi_req_scan_result(scan_request_index);
			scan_request_index++;
		} 
		else 
		{
			//printf("can not find AP %s\r\n", MAIN_WLAN_SSID);
			__nop();
			
			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			//printf("Wi-Fi disconnected\r\n");
			WINC_WIFI_is_conn = 0;
			/* Request scan. */
			//m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		//printf("Wi-Fi connected\r\n");
		sprintf(errout,"Wi-Fi IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);

		WINC_WIFI_is_conn = 5;
		osSignalSet(WINC_WIFI_Thread_ID, WINC_WIFI_CONNECT_DONE);
		
	// Resolve Server URL.
	//err = gethostbyname((uint8*)host_name);
		break;
	}

	default:
	{
		break;
	}
	}
}





void WINC1500_setup(void)
 {

	 char device_name[] = {"FlixiBadge"};
//	 uint8_t mac_adr[6] = {0,0,0,0,0,0};

	 
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();

  /* Configure the system clock */
//  SystemClock_Config();

  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//	 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
//	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
  //MX_SPI3_Init();
	
  /* USER CODE BEGIN 2 */

	winc_spi_mutex_id = osMutexCreate(osMutex(winc_spi_mutex));

	/* Initialize the board. */
//	system_init();

	/* Initialize the UART console. */
	//configure_console();
	//printf("HELLO....");

	/* Initialize the BSP. */
		WINC_WIFI_reset_init();

		socketInit();
		registerSocketCallback(tcpClientSocketEventHandler, dnsResolveCallback);
//	m2m_wifi_get_mac_address(mac_adr);
	/* Request scan. */
		m2m_wifi_set_device_name(device_name,strlen(device_name));
//	m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
//	WINC_busy = 5;
	//gethostbyname

//	while(WINC_busy){};
	
	WINK_ISR_H_task_Thread_ID = osThreadCreate(osThread(WINK_ISR_H_task),NULL);
	
//	while (1) {
//		/* Handle pending events from network controller. */
//		while (m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
//		}
//	}

//	return 0;
}



void WINK_ISR_H_task(void const *p){
	while (1) {
		/* Handle pending events from network controller. */
		osMutexWait(winc_spi_mutex_id, osWaitForever);
		while (m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
		}
		osMutexRelease(winc_spi_mutex_id);
		//osThreadYield();
		osSignalWait(WINC_ISR, osWaitForever);
	}
}


void WINC_http_socket_send(void *pvSendBuffer, uint16_t u16SendLength){
	osEvent evt;
	
		osMutexWait(winc_spi_mutex_id, osWaitForever);
		clientSocket_FTP_HTTP_1_Thread_ID = osThreadGetId();
		send(clientSocket_FTP_HTTP_1, pvSendBuffer, u16SendLength, 0);
		osMutexRelease(winc_spi_mutex_id);	
	
		evt = osSignalWait(SEND_DONE_SIG, osWaitForever);	
//		switch(evt.status){
//			case osEventSignal:
//				return;
//			case osEventTimeout:
//				LED_on(LED_GREEN);
//				break;
//			default:
//				break;
//		}//switch(evt.status)
	
}

void WINC_http_socket_send_unblocking(void *pvSendBuffer, uint16_t u16SendLength){
		osMutexWait(winc_spi_mutex_id, osWaitForever);
		clientSocket_FTP_HTTP_1_Thread_ID = osThreadGetId();	
		send(clientSocket_FTP_HTTP_1, pvSendBuffer, u16SendLength, 0);
		osMutexRelease(winc_spi_mutex_id);
}

int WINC_http_socket_open_by_ip(uint32_t ip, uint16_t port){
	struct sockaddr_in strAddr;
	TCPconnRESULT = TCP_FAILED;

			clientSocket_FTP_HTTP_1 = socket(AF_INET,SOCK_STREAM,0);
			if(clientSocket_FTP_HTTP_1 >= 0){
				strAddr.sin_family = AF_INET;
				strAddr.sin_port = _htons(port);
				strAddr.sin_addr.s_addr = _htonl(ip);
				do{
					osMutexWait(winc_spi_mutex_id, osWaitForever);
					clientSocket_FTP_HTTP_1_Thread_ID = osThreadGetId();					
					connect(clientSocket_FTP_HTTP_1, (struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));
					osMutexRelease(winc_spi_mutex_id);
					osSignalWait(CONNECT_DONE_SIG, osWaitForever);
				}while(TCPconnRESULT != TCP_DONE);
			}
	return clientSocket_FTP_HTTP_1;
}

void WINC_http_socket_open(char* host_name, uint16_t port){
	int8_t err;
	osEvent evt;
	TCPconnRESULT = TCP_FAILED;

	while(1){
	do{
		do{
			osMutexWait(winc_spi_mutex_id, osWaitForever);
			clientSocket_FTP_HTTP_1_Thread_ID = osThreadGetId();
			clientSocket_FTP_HTTP_1_port = port;

			err = gethostbyname((uint8*)host_name);
			osMutexRelease(winc_spi_mutex_id);
			
		}while(err != SOCK_ERR_NO_ERROR);
		evt = osSignalWait(CONNECT_DONE_SIG, 1000);
	}while(TCPconnRESULT != TCP_DONE);
			switch(evt.status){
				case osEventSignal:
						return;
				//case osEventTimeout:

				default:
					break;
					//return ERR_OTHERSIGNALERROR;
		}//switch(evt.status)
	}	
}
void WINC_http_socket_close(int socket)
{
	//clientSocket_FTP_HTTP_1_Thread_ID = osThreadGetId();
	osMutexWait(winc_spi_mutex_id, osWaitForever);
	close(clientSocket_FTP_HTTP_1);
	osMutexRelease(winc_spi_mutex_id);
	osDelay(10);
	//osSignalWait(CLOSE_DONE_SIG, osWaitForever);
}
int16_t WINC_http_socket_recv(void *pvRecvBuf, uint16 u16BufLen, uint32 u32Timeoutmsec){				
	clientSocket_FTP_HTTP_1_recv_umt = 0;
	osMutexWait(winc_spi_mutex_id, osWaitForever);
	clientSocket_FTP_HTTP_1_Thread_ID = osThreadGetId();	
	recv(clientSocket_FTP_HTTP_1, pvRecvBuf, u16BufLen, u32Timeoutmsec);
	osMutexRelease(winc_spi_mutex_id);
	osSignalWait(RECV_DONE_SIG, osWaitForever);
	return clientSocket_FTP_HTTP_1_recv_umt;
}
void WINC_http_socket_recv_unblocking(void *pvRecvBuf, uint16 u16BufLen, uint32 u32Timeoutmsec){				
	clientSocket_FTP_HTTP_1_recv_umt = 0;
	osMutexWait(winc_spi_mutex_id, osWaitForever);
	clientSocket_FTP_HTTP_1_Thread_ID = osThreadGetId();	
	recv(clientSocket_FTP_HTTP_1, pvRecvBuf, u16BufLen, u32Timeoutmsec);
	osMutexRelease(winc_spi_mutex_id);
	//osSignalWait(RECV_DONE_SIG, osWaitForever);
	//return clientSocket_FTP_HTTP_1_recv_umt;
}
uint16_t WINC_http_socket_recv_block_unblocking(void){				
	//clientSocket_FTP_HTTP_1_Thread_ID = osThreadGetId();
	//recv(clientSocket_FTP_HTTP_1, pvRecvBuf, u16BufLen, u32Timeoutmsec);
	osSignalWait(RECV_DONE_SIG, osWaitForever);
	return clientSocket_FTP_HTTP_1_recv_umt;
}


//void WINC_WIFI_new_connect(char* ssid,char* pass){
//	WINC_WIFI_Thread_ID = osThreadGetId();	
//	strcpy(_ssid,ssid);
//	strcpy(_pass,pass);
//	
//osMutexWait(winc_spi_mutex_id, osWaitForever);
//	m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
//osMutexRelease(winc_spi_mutex_id);
//	
//	osSignalWait(WINC_WIFI_CONNECT_DONE, osWaitForever);	
//}
//void WINC_WIFI_def_connect(void){
//	WINC_WIFI_Thread_ID = osThreadGetId();	
//osMutexWait(winc_spi_mutex_id, osWaitForever);
//	m2m_wifi_default_connect();
//	osMutexRelease(winc_spi_mutex_id);
//	osSignalWait(WINC_WIFI_CONNECT_DONE, osWaitForever);	
//}

void WINC_WIFI_new_connect_unblocking(char* ssid,char* pass){

	osMutexWait(winc_spi_mutex_id, osWaitForever);
	WINC_WIFI_Thread_ID = osThreadGetId();	
	m2m_wifi_connect(ssid,strlen(ssid),MAIN_WLAN_AUTH,pass,M2M_WIFI_CH_ALL);
	osMutexRelease(winc_spi_mutex_id);
}


uint8_t WINC_WIFI_new_connect(char* ssid,char* pass,uint32_t timeout_millisec){

	osMutexWait(winc_spi_mutex_id, osWaitForever);
	WINC_WIFI_Thread_ID = osThreadGetId();	
	m2m_wifi_connect(ssid,strlen(ssid),MAIN_WLAN_AUTH,pass,M2M_WIFI_CH_ALL);
	osMutexRelease(winc_spi_mutex_id);
	return(osSignalWait(WINC_WIFI_CONNECT_DONE, timeout_millisec));	
}


uint8_t WINC_WIFI_new_connect_with_scan(char* ssid,char* pass,uint32_t timeout_millisec){

	strcpy(_ssid,ssid);
	strcpy(_pass,pass);

	osMutexWait(winc_spi_mutex_id, osWaitForever);
	WINC_WIFI_Thread_ID = osThreadGetId();		
	m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
	osMutexRelease(winc_spi_mutex_id);
	
	return(osSignalWait(WINC_WIFI_CONNECT_DONE, timeout_millisec));	
}

uint8_t WINC_WIFI_new_connect_with_scan_list(WifiNetworkListItem* network_list, uint8_t network_list_count )
{

	wifi_network_allowed_list = network_list;
	wifi_network_allowed_list_count = network_list_count;

	osMutexWait(winc_spi_mutex_id, osWaitForever);
	WINC_WIFI_Thread_ID = osThreadGetId();		
	m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
	//osDelay(1000);
	osMutexRelease(winc_spi_mutex_id);
	
	//return(osSignalWait(WINC_WIFI_CONNECT_DONE, timeout_millisec));	
}

uint8_t WINC_WIFI_def_connect(uint32_t timeout_millisec){

	osMutexWait(winc_spi_mutex_id, osWaitForever);
	WINC_WIFI_Thread_ID = osThreadGetId();	
	m2m_wifi_default_connect();
	osMutexRelease(winc_spi_mutex_id);
	
	return(osSignalWait(WINC_WIFI_CONNECT_DONE, timeout_millisec));	
}









uint8_t WINC_WIFI_is_connected(void){
	return WINC_WIFI_is_conn;
}
void WINC_WIFI_reset_init(void){
	int8_t ret;
	tstrWifiInitParam param;
	
	WINC_WIFI_is_conn = 0;
//	nm_bsp_init();

	
	/* Initialize Wi-Fi parameters structure. */
	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		//printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		__nop();
		while (1) {
		}
	}
}

//void WINC_WIFI_reset_init2(void){
//	int8_t ret;
//	tstrWifiInitParam param;
//	
//	WINC_WIFI_is_conn = 0;
//	socketDeinit();
//	
//	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

//	/* Initialize Wi-Fi driver with data and status callbacks. */
//	param.pfAppWifiCb = wifi_cb;
//	ret = m2m_wifi_init(&param);
//	if (M2M_SUCCESS != ret) {
//		//printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
//		__nop();
//		while (1) {
//		}
//	}
//	
//			socketInit();
//			registerSocketCallback(tcpClientSocketEventHandler, dnsResolveCallback);

//}
