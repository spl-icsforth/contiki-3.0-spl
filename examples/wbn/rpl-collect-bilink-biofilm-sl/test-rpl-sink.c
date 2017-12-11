/*
 * Example program collect sink application. A set of sensors are sampled periodically, 
 * with sampling period Ts and the data are sent over a udp link to the sink.
 * SINK id should be 1, so SOURCE nodes should have id >1
 * THIS is THE SINK application.
 * Whenever data are received, Green Led toggles and data are printed in uint8.
 * n.panousopoulou@gmail.com
 * Date created: 2013-04-18 
 * Date last modified: 2013-05-06 (defining two ways for accessing the serial port, using the second)
 *                     2013-05-24: Defining new serial packet format:
 *                     --------------------------------------------------  
 * 					   |BOF| Source ADDR| Payload Length | Payload | EOF|      
 *                     --------------------------------------------------
 *                     BOF: 192
 *                     EOF: 193                
 * 
 * 					   2013-06-03: RSSI - LQI per packet added.	  
 * //convert to sensor_datamsg (look at test-udp-source for definition of data structure)
	
	    msg->light1 = data2send[1]<< 8 | data2send[0];
		msg->light2 = data2send[3]<<8 | data2send[2];
		msg->temp = data2send[5]<<8 | data2send[4];
		msg->humm = data2send[7]<<8 | data2send[6];
	 */
	 


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "dev/cc2420/cc2420.h"

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
//#include "net/uip.h"
#include "net/rpl/rpl.h"
#include "net/linkaddr.h"
//#include "net/rime/rimeaddr.h"
#include "lib/random.h"


#include "net/netstack.h"
#include "dev/button-sensor.h"
#include "dev/serial-line.h"
#include "dev/leds.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "nm-common.h"



#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688



//integration with serial line communication
//NOTE: THE SERIAL DOWNLINK COMMUNICATION IS NOT COMPATIBLE WITH xm1000
#ifdef WITH_DL
#define CMD_TIMER 5*CLOCK_SECOND
#define SERIAL_BUF_SIZE 128
#define START_BYTE 192
#define STOP_BYTE 193
#define LENGHT_OFFSET 2
#define WAIT_FOR_SYNCH 0
#define IN_SYNCH 1
#define LEN_MIN 9
#endif //WITH_DL

/* DEFINE DEBUG  for print messages */

#define DEBUG DEBUG_PRINT

#include "net/ip/uip-debug.h"

static struct uip_udp_conn *server_conn;
PROCESS(udp_server_process, "UDP server process");
#ifndef WITH_DL
	AUTOSTART_PROCESSES(&udp_server_process, &nm_common_process);
#else
	PROCESS(udp_sendcmd_process, "UDP Send CMD process");
	AUTOSTART_PROCESSES(&udp_server_process, &udp_sendcmd_process, &nm_common_process);
#endif  //WITH_DL



//functions definition
void serial_tx_data(void * data2send, int datalen);

static uint16_t do_rssi(void);
static uint8_t do_lqi(void);
static struct mac_counters get_mac_counters(void);
void msend(void *c);



static uint16_t rssi;
static uint8_t lqi;
static uint8_t mynf;


//static rimeaddr_t src_addr;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#ifdef WITH_DL
struct cmd_request_t {
	uint8_t cmd_type; //bcast or unicast
	
	uint16_t cmd_msn;
	
	uint8_t htl;
	
	uint8_t cmd_index;	//the value of the cmd - used for processing
	
	uint8_t cmd_parameter[4];
	
	uint8_t trace[MAX_NEIGHS]; // i can afford the space...
	
	uint8_t trace_len;
} cmd_request_t;

static char rxbuf[SERIAL_BUF_SIZE];
static uint8_t cnt,state;
static process_event_t event_data_ready;
void process_incoming_packet(char *incoming_packet, uint8_t size);
#endif  //WITH_DL	
//static struct cmd_t *cc;	

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


 static void
tcpip_handler(void)
{
  uint8_t *appdata;
  linkaddr_t sender;
// #ifndef SERIAL_MODE
	#ifdef NM
		struct mac_counters txrxcounters;
	#endif
 // #endif
 // uint8_t seqno;
 // uint8_t hops;
		int i;
  
  if(uip_newdata()) {
	leds_toggle(LEDS_GREEN);
	//2013-05-06 @nancypan
	//method#1: transfer over the serial port (decimal values are treated not as ascii characters!)
    /*e.g. the value 192 will be transferred as 49 (='1') 57(='9') 50(='2')
     * 
     
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */ 
    appdata = (uint8_t *)uip_appdata;
    rssi = do_rssi();	
    lqi = do_lqi();
    mynf = get_nf();
    
	 #ifdef NM	
		txrxcounters=get_mac_counters();
	 #endif	
    //this is the piece of rx data in uint8t format
    //this is the id of the sender (as defined in compile time).
    #ifndef SERIAL_MODE
        printf("192 %d %d ", UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 2], uip_datalen());
		for (i=0;i<uip_datalen();i++)
		{
			printf("%d ",appdata[i]);
		
		}
		printf("%d %d %d %d ", (char)cc2420_get_channel(),rssi,lqi, mynf);
		
	    #ifdef NM
			printf("%d ", txrxcounters.txcnt);
			printf("%u ", txrxcounters.listaddr);
			for (i=0;i<MAX_NEIGHS;i++){
				printf("%d ", txrxcounters.rxcnt[i]);			
			}
		#endif
		printf("193\n");
    
    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //method#2: transfer over the serial port (decimal values are not treated not as ascii characters!)
    
    /*this will transfer ONLY the payload of the packet. In order to transfer additional information,
    * the input arguments of the serial_tx_data need to be accordingly adjusted.
    * (see test-blink.c for more details)
    */
    //note: this does not work on cooja!
    #else
		if (uip_datalen()>0) {
		serial_tx_data(appdata,uip_datalen());
		}
    #endif 
  }
}

//UART Transmit..
void serial_tx_data(void * data2send, int datalen){
  	
  	u8_t *data_ptr;
 	unsigned int i=0;
 	#ifdef NM
		struct mac_counters txrxcounters;
		txrxcounters=get_mac_counters();
 	#endif
 	
 	data_ptr=(uint8_t *)data2send;
 	

 	
 	slip_arch_writeb((char)192); //start of serial frame
	
 	//this is the id of the sender:
 	slip_arch_writeb((char)UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 2]);//src_addr.u8[0]);
 	//this is the payload length + metadata_length (if any): (in BYTES!)
 	#ifdef NM
		//@2014-05-08>>>>>>>>>>>>>>>>>
		//slip_arch_writeb((char)datalen+4+NET_SIZE+1);
		//slip_arch_writeb((char)datalen+4+NET_SIZE+2); // added a field for noise floor...
		if (datalen <=65){
			slip_arch_writeb((char)datalen+5+MAX_NEIGHS+3); // added a field for noise floor...
		}
		else {
			slip_arch_writeb((char)datalen);	
		}
		//<<<<<<<<<<<<<<<<<<<<<<<<<<<
 	#else
		//@2014-05-08>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
		//slip_arch_writeb((char)datalen+4);
		if (datalen<=65){
			slip_arch_writeb((char)datalen+5); // added a field for noise floor
		}
		else {
			slip_arch_writeb((char)datalen);
			
		}	
		//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 	#endif
	
 	while (i<datalen){
		
		slip_arch_writeb((char)*data_ptr);
		data_ptr++;
		i++;
	}
	if (datalen<=65){
		slip_arch_writeb((char)(cc2420_get_channel() & 0xff));//lsb
		slip_arch_writeb((char)(rssi & 0xff)); //lsb
		slip_arch_writeb((char)(rssi >> 8)); //msb
	
		slip_arch_writeb((char)lqi);
		slip_arch_writeb((char)mynf);
	}
	//the tail: tx & rx counters at the mac layer...
	#ifdef NM
	if (datalen<=65){
		slip_arch_writeb((char)txrxcounters.txcnt);
		slip_arch_writeb((char)(txrxcounters.listaddr)<<8);//msb
		slip_arch_writeb((char)(txrxcounters.listaddr)& 0xff);//lsb
		for (i=0;i<MAX_NEIGHS;i++){
			slip_arch_writeb((char)txrxcounters.rxcnt[i]);			
		}
	}	
	#endif
	slip_arch_writeb((char)193); //end of serial frame

	return;
}



static uint16_t
do_rssi(void)
{
 	return packetbuf_attr(PACKETBUF_ATTR_RSSI);
  
}

static uint8_t
do_lqi(void)
{
	return (uint8_t)packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);	
}
#ifdef NM
static struct mac_counters get_mac_counters()
{
	
	return NETSTACK_RDC.get_txrx_counters();
		
}

/*---------------------------------------------------------------------------*/
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//2014-05-08 nancy @ FORTH-ICS

#ifdef WITH_DL

//static void send_command_request(uip_ipaddr_t *destip, struct cmd_request_t *cmd_request) 
//void send_command_request(struct cmd_request_t *cmd_msg, uip_ipaddr_t *destip)
void send_command_request(struct cmd_request_t *aa, uip_ipaddr_t *destip)
{
	//convert cmd_request to packet buffer and send
	//uip_ipaddr_t *destip = (uip_ipaddr_t *) c;	
	const char *msg;//[sizeof(struct cmd_request_t)];
//	memcpy(msg, &cmd_msg, sizeof(struct cmd_request_t));

//	printf("\ncmd 2send: %d, %d, %d, %d",aa->cmd_type,
//	aa->cmd_msn, aa->cmd_index, aa->htl);
	msg = (const char *)aa;
		 
	if (destip == NULL) 
	{		
		//this is a broadcast packet for everybody in my range.	

		uip_create_linklocal_allnodes_mcast(&server_conn->ripaddr);	
		uip_udp_packet_send(server_conn, msg, sizeof(struct cmd_request_t));
		uip_create_unspecified(&server_conn->ripaddr);	
	}
	else 
	{
		uip_udp_packet_sendto(server_conn, msg, sizeof(struct cmd_request_t), destip, UIP_HTONS(UDP_CLIENT_PORT));	
	}	
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#endif
#endif
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  uip_ipaddr_t ipaddr;
  struct uip_ds6_addr *root_if;

  //  cc2420_set_channel(21);

  PROCESS_BEGIN();

  PROCESS_PAUSE();

  

  PRINTF("UDP server started\n");

#if UIP_CONF_ROUTER
  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, SINK_ID);
//uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
//  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
  
  root_if = uip_ds6_addr_lookup(&ipaddr);
  
  if(root_if != NULL) 
  {
    rpl_dag_t *dag;
    dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr);
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr, 64);
    PRINTF("created a new RPL dag\n");
    
  } 
  else 
  {
	  PRINTF("failed to create a new RPL DAG\n");\
  }
  
#endif /* UIP_CONF_ROUTER */

  /* The data sink runs with a 100% duty cycle in order to ensure high
     packet reception rates. */
  NETSTACK_RDC.off(1);

  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  if (server_conn == NULL)
  {
	  printf("No UDP connection available, exiting PROCESS");
	  PROCESS_EXIT();
  }
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));
  
  // Debug messages for udp connection

  PRINTF("Created a server connection with remote address ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(server_conn->lport),
         UIP_HTONS(server_conn->rport));
         
  
  
  while(1) 
  {
	PROCESS_YIELD();
	if(ev == tcpip_event)
	{
		nm_input();
		tcpip_handler();
	}
  }
  

  PROCESS_END();
}

#ifdef WITH_DL

static int uart_rx_callback(unsigned char c)
{
	
	 if (state == WAIT_FOR_SYNCH) {
		 if ((uint8_t)c == START_BYTE) {
			 state = 1-state;
			 cnt = 0;
			 }
		 
		 
		 }
		 
	 else if (state == IN_SYNCH)	 {
		 if ((uint8_t)c == STOP_BYTE) {
			if (cnt < LEN_MIN) {
				cnt = 0;
			}
			else 
			{
				//format: <addr> <length> <data>
				if (cnt == rxbuf[LENGHT_OFFSET] +2)
				{
					
					//process incoming packet
					process_incoming_packet(rxbuf, cnt);
					cnt = 0;
					state = 1-state;
					
				}
				else {
					rxbuf[cnt++] = c;
					 if(cnt >= SERIAL_BUF_SIZE) {
                          
                            cnt = 0;
                          
                            state = WAIT_FOR_SYNCH;
                        }
					}
			}
			 
		 }
		 else {
			 rxbuf[cnt++] = c;
			 if(cnt >= SERIAL_BUF_SIZE) {
                 cnt = 0;
                 state = WAIT_FOR_SYNCH;
             }
		}
	 }
}

void process_incoming_packet(char *incoming_packet, uint8_t size)
{
	uint8_t *tmp;
	uint8_t kk; 
	
	
	tmp = (uint8_t *)incoming_packet;
	
/*	printf("\nReceived:");
	for (kk=0;kk<size;kk++) {
		printf("%u ", tmp[kk]);
		
	}*/
	//incoming packets have the following format: 
	//<address> <length of message> <length of message> message
	
	//241: the index of a CMD_REQUEST from MServer(hardcoded)
	//242: the index of a CMD_Indication from Gateway via MServer


	if (tmp[3] != 241 && tmp[3] !=242) {
		
		return;
		}
	else {
		uint8_t startofmsg;
		static struct cmd_request_t new_cmd;
		
		startofmsg = 3;
		new_cmd.cmd_type = tmp[startofmsg++];	
		new_cmd.cmd_msn = (tmp[startofmsg] <<8) | tmp[startofmsg+1];
		startofmsg = startofmsg+2;
		new_cmd.cmd_index = tmp[startofmsg++];
		
		for (kk=startofmsg;kk<size;kk++){
			new_cmd.cmd_parameter[kk-startofmsg] = tmp[kk];
			
			}
		
		
		new_cmd.htl = MAX_HTL;
	
		new_cmd.trace[0] =linkaddr_node_addr.u8[7];
		for (kk=1;kk<MAX_NEIGHS;kk++){
			new_cmd.trace[kk] = 0;
			}
		new_cmd.trace_len = 1;
					
		
		process_post(&udp_sendcmd_process, event_data_ready, &new_cmd);
	}	
	
	

}

PROCESS_THREAD(udp_sendcmd_process, ev, data){
	
	 
  static uint8_t tl,ii; //if not static, no stable!
 
 
  static struct etimer ee;

   uip_ipaddr_t tmp_ipaddr;
 static	struct cmd_request_t cmd_msg;
 
 
 
	PROCESS_BEGIN();

	PROCESS_PAUSE();

	cnt = 0;
	state = 0;
	//the callback function for the serial line communication
	uart1_set_input(uart_rx_callback);


   //SENSORS_ACTIVATE(button_sensor);
  //started = 0;       
 
	//build the cmd request msg.
	while (1) {	
		PROCESS_YIELD();
		
		if (ev == event_data_ready && data !=NULL) {
			memcpy(&cmd_msg, data, sizeof(struct cmd_request_t));
			leds_toggle(LEDS_BLUE);
			
		  //  tl =get_nmlist_length();
		
		tl=0;
			
		
		//you have no neigbours at the nm plane
		//   if (tl == 0){
		    //at this level you dont have any neighbours at the nm level, so broadcast
				//cmd_msg.cmd_type = 255;				
				send_command_request(&cmd_msg, NULL);
		
			
		//	}
			/*else {
				//cmd_msg.cmd_type = 241;
				static uip_lladdr_t tmplist[MAX_CHILDREN];
				get_nmlist(tmplist, tl);
			//	printf("preparing to send to: ");
				for (ii=0; ii<tl;ii++)
				{
		
			//		 printf("%d.%d.%d.%d.%d.%d:: ", tmplist[ii].addr[0],tmplist[ii].addr[1],
			//		 tmplist[ii].addr[2],tmplist[ii].addr[3],tmplist[ii].addr[4],tmplist[ii].addr[5]);
					//generate address from the link local addresse.g. for node 2: uip_ip6addr(&tmp_ipaddr, 0xfe80, 0, 0, 0, 0x212, 0x7402, 0x02,0x202);	
					//use the global reference
					 uip_ip6addr(&tmp_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
					 uip_ds6_set_addr_iid(&tmp_ipaddr,&tmplist[ii]);
					 uip_ds6_addr_add(&tmp_ipaddr, 0, ADDR_MANUAL);
		

					 
					 send_command_request(&cmd_msg,&tmp_ipaddr);
					 etimer_set(&ee,random_rand()%CLOCK_SECOND);
					 PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&ee));
					 
                   
				 }
			//	 printf("\n");
				// free(tmplist);
				}*/
			//started = 1-started;
			
			
		}
		
	}
	    PROCESS_END();
	}

 #endif	
	
/**********************************************************/
