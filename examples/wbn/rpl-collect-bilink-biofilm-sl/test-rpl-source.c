/*
 * Example program collect source application using RPL. A set of sensors are sampled periodically, 
 * with sampling period Ts and the data are sent over a udp link to the sink.
 * SINK id should be 20, so SOURCE nodes should have id >=1.
 * THIS is THE SOURCE application.
 * 
 * n.panousopoulou@gmail.com
 * Date last modified: 2014-06-xx
 * Route traces added to the data structure.
 * 2013-11-26: RPL compatible, with using UVEG's API for measuring noise floor. Data structure updated.
 * 2014 downlink implemented - dual sensing / relaying functionality
 * September 2014: Final version
 *                       
 */

#include "contiki.h"
#include "contiki-net.h"
#include "uip.h"
#include "dev/leds.h"
#include "node-id.h"

#include "dev/light-sensor.h"
#include "dev/sht11/sht11-sensor.h"


#include "dev/battery-sensor.h"
#include "dev/button-sensor.h"
#include "dev/cc2420/cc2420.h"
//#include "net/rime.h"
//#include "net/netstack.h"


#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
//What is this?
//#include "net/neighbor-info.h"
#include "net/rpl/rpl.h"
#include "nm-common.h"
#include "dev/button-sensor.h"



#include <stdio.h>
#include <string.h>
#include "dev/uart0.h"


#include "lib/list.h"
#include "lib/memb.h"


#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

#define UDP_MOUT_PORT 1234




#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])


#define SEND_INTERVAL	6//0//9
//#define SENSFACTOR	6//0//10
#define SEND_TIME	(random_rand() % (SEND_INTERVAL* CLOCK_SECOND))


//#define DEBUG DEBUG_PRINT
//#include "net/uip-debug.h"

static struct uip_udp_conn *client_conn;
static struct uip_udp_conn *c2c_conn;

//static struct unicast_conn unicast;

static uip_ipaddr_t server_ipaddr;
static uint16_t previous_msn, msn;


static process_event_t event_data_ready, cmd_event, start_event, tx_cmd_event;
//-------------------------------------------------------------
//for connectivity with arduino via uart0
//the length of the buffer size
#define SERIAL_BUF_SIZE 128//256
//the start byte
#define START_BYTE 192
//the stop byte
#define STOP_BYTE 193

#define LENGHT_OFFSET 1//2

#define WAIT_FOR_SYNCH 0
#define IN_SYNCH 1
//minimum length of uart0 packets.
#define LEN_MIN 9


//incoming data are stored here
static unsigned char rxbuf[SERIAL_BUF_SIZE];
//and this is the cnt of packets and the status of the uart0 receiver...
static uint8_t cnt,state,uart_data_ready, sensor_mode, msg_index;
uint8_t process_incoming_packet(unsigned char *incoming_packet, uint8_t size);
static float send_interval;
struct biofilm_datamsg{
	uint8_t msg_index; //the type of message and the type of sensor
	uint8_t totalnumofchannels;
	uint8_t freq_index[MAX_FREQS];
	uint16_t real_part[MAX_FREQS];
	uint16_t img_part[MAX_FREQS];
	uint8_t temp[MAX_FREQS];
	uint16_t battery_level;
	uint16_t ptx; //transmission power.
	uint16_t noise_level; //mean accross all channels or per specific channel of transmission.	
	uint16_t num;
	
}biofilm_datamsg;

/* This structure holds information about the biofilm data. */
struct pck2send {
  /* The ->next pointer is needed since we are placing these on a
Contiki list. */
  struct pck2send   *next;

  /* The ->addr field holds the LL address of the child. */
  struct biofilm_datamsg data2send;
  
 // rimeaddr_t rimeaddr;
  
  //the time out timer for removing old entries
  struct ctimer ctimer;

};


/* This MEMB() definition defines a memory pool from which we allocate
packet entries entries. */
MEMB(pck2send_memb, struct pck2send, 10);

/* The packets2send_list is a Contiki list that holds the packets pending for sending.*/
LIST(pck2send_list);

static void update_pck2send(struct biofilm_datamsg *data2send);
static void biofilmdata_send(void *n);

//---------------------------------------------------------------

struct sensor_datamsg{
	#ifdef NM
	#ifdef TRACE_ROUTE
		#ifdef TRACE_LQ
			uint16_t route_link[MAX_NEIGHS-2]; //MAX_NEIGHS-sink-current node
			uint8_t route_nf[MAX_NEIGHS-2]; //MAX_NEIGHS-sink-current node
		#endif //TRACE_LQ
	
		uint16_t route_trace[2]; //this contains route-specific details	
		
	#endif //TRACE_ROUTE
		uint8_t node_tx_counter;
		uint16_t listaddr;
		uint8_t node_rx_counter[MAX_NEIGHS];
	#endif //NM
	uint16_t light1;
	uint16_t light2;
	uint16_t temp;
	uint16_t humm;
	uint16_t battery_level;
	uint16_t ptx; //transmission power.
	uint16_t noise_level; //mean accross all channels or per specific channel of transmission.	
	uint16_t num;

	
}sensor_datamsg;



struct cmd_request_t {
	uint8_t cmd_type; //bcast or unicast
	
	uint16_t cmd_msn;
	
	uint8_t htl;
	
	uint8_t cmd_index;	//the value of the cmd - used for processing
	
	uint8_t cmd_parameter[4];
	
	uint8_t trace[MAX_NEIGHS]; // i can afford the space...
	
	uint8_t trace_len;
} cmd_request_t;

union ff{
	float f;
	unsigned char b4[sizeof(float)];
	
	}ff;

struct sensor_datamsg msg;	

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
PROCESS(c2c_process, "C2C process");
PROCESS(sensing_process, "Sensing process");
PROCESS(biofilm_sensing_process, "biofilm process");
PROCESS(uart_connection_error_handling, "uart error handling process");


#ifdef NM
 #ifdef TRACE_ROUTE
	#ifdef TRACE_LQ
		AUTOSTART_PROCESSES(&udp_client_process, &sensing_process, &biofilm_sensing_process, &nm_common_process,&c2c_process, &uart_connection_error_handling);//,&c2c_relay_process);
	#endif	
 #endif
 #else
	AUTOSTART_PROCESSES(&udp_client_process, &sensing_process, &biofilm_sensing_process, &c2c_process,&uart_connection_error_handling);//,&c2c_relay_process);
#endif

/**********************************************************************/
static void
tcpip_handler(void)
{
 
  if(uip_newdata()) {
	
	//leds_toggle(LEDS_GREEN);
	  //if a receive a command_request with an msn that i have replied to before or with hop counter == 0, ignore 
    /* Ignore incoming data */
    //else 
   
    uint8_t i;
     uint8_t *cmd_type;
	struct cmd_request_t *appdata2;
//	struct ctimer ctimer;
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */ 
  
    cmd_type = (uint8_t *)uip_appdata;
   // 
  
    if (cmd_type[0] >240 && uip_datalen() == sizeof(struct cmd_request_t))
    {
		//memcpy(&appdata,(struct cmd_request_t *)uip_appdata, sizeof(struct cmd_request_t));
		appdata2 = (struct cmd_request_t *)uip_appdata;
		
	
		if (appdata2->htl>=0 && appdata2->cmd_msn != previous_msn) {
	   
		//1. read command and process command
		//note: for this version: start and stop data collection
		//	 printf("*********%d, %d, %d\n", cmd_type[0],uip_datalen(),sizeof(struct cmd_request_t));
				
		process_post(&sensing_process, cmd_event, appdata2);
		//2. initiate forward process
		 previous_msn = appdata2->cmd_msn;
		//forward_cmd(appdata2);
		for (i=0;i<appdata2->trace_len;i++)
		{
			if (appdata2->trace[i] == 0){
				
				break;	
			}
			
	//		printf("%d, ",dd->trace[i]);
		
		}
		
	//	printf("\n");
		appdata2->trace[i] = linkaddr_node_addr.u8[7];
		appdata2->trace_len += 1; 
		appdata2->htl-= 1;
		
		//and now relay into the network:
		//leds_toggle(LEDS_BLUE);
		process_post(&c2c_process,event_data_ready, appdata2);
	
		}
	
	}	
 }
}


/**********************************************************************/
void send_command_request(void *c)
{
	const char *msg;
	struct cmd_request_t *cc;
	
	cc = (struct cmd_request_t *)c;
	//printf("at send: %d, %d, %d\n", cc->cmd_msn, cc->htl, cc->cmd_index);
	//uip_ipaddr_t *destip = (uip_ipaddr_t *)c;	
	
	msg = (const char *)cc; 
	//if (destip == NULL) {
	if (cc->cmd_type == 241 || cc->cmd_type == 242) {		
	//this is a broadcast packet for everybody in my range. (relaying a command request)	
		uip_create_linklocal_allnodes_mcast(&c2c_conn->ripaddr);
		
		uip_udp_packet_send(c2c_conn, msg, sizeof(struct cmd_request_t));
		uip_create_unspecified(&c2c_conn->ripaddr);
	}
	
	else if (cc->cmd_type == 240 || cc->cmd_type == 239){
		//this is a reply just for the mserver or the gateway via the mserver
		uip_udp_packet_sendto(client_conn, msg, sizeof(struct cmd_request_t),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
	}	
	
	//else {
	//	printf("Unrecognized cmd index! Exiting...\n");
	//}	

	//	free(msg);
}

static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  /* set server address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, SINK_ID);

}
	
/*--------------------------------------------------------------------*/
void
collect_common_send(void)
{
 
   static int ii,j,k;
   static uint8_t data2send[sizeof(sensor_datamsg)];


  if(client_conn == NULL) {
    /* Not setup yet */
    return;
  }
  
    //manual conversion...
       ii = 0;
       k=0;
		#ifdef NM
			#ifdef TRACE_ROUTE
				#ifdef TRACE_LQ
					j=0;
					for (ii=0;ii<(MAX_NEIGHS-2)*2;ii=ii+2)
					{
						data2send[ii] = msg.route_link[j] & 0xff;
						data2send[ii+1] = msg.route_link[j] >> 8;
						j++;
					}
					j=0;
					for (ii=ii;ii<3*(MAX_NEIGHS-2);ii++)
					{
						data2send[ii] = msg.route_nf[j] & 0xff;
						j++;
					}
				k = ii;	
				#endif //trace lq
	
		//route tracing.....
			j=0;
					for (ii=ii;ii<k+4;ii=ii+2)
					{
						if ( ii % 2 == 0){ 
							data2send[ii] = msg.route_trace[j] >>8;
							data2send[ii+1] = msg.route_trace[j] & 0xff;
							j++;
						}		
					}
				
				
	
			k=ii;	
			
			#endif //trace route
			struct mac_counters tmpcnt;
			tmpcnt = NETSTACK_RDC.get_txrx_counters();
			
		    data2send[ii++] = tmpcnt.txcnt;
		    k++;
		    data2send[ii++] = tmpcnt.listaddr>>8;//MSB
		    k++;
		    data2send[ii++] = tmpcnt.listaddr & 0xff;//LSB
		    //i++
		    k++;
		    for (ii=ii;ii<k+MAX_NEIGHS;ii++) 
		    {
			  	
			  data2send[ii] = tmpcnt.rxcnt[ii-k];
			  
			}
		#endif //NM
		data2send[ii] = msg.light1 & 0xFF; //lsb
		data2send[ii+1] = msg.light1 >> 8; //msb
		
		data2send[ii+2] = msg.light2 & 0xFF;
		data2send[ii+3] = msg.light2 >> 8;
		
		data2send[ii+4] = msg.temp & 0xFF;
		data2send[ii+5] = msg.temp >> 8;
		
		data2send[ii+6] = msg.humm & 0xFF;
		data2send[ii+7] = msg.humm >> 8;
		
		data2send[ii+8] = msg.battery_level & 0xFF;
		data2send[ii+9] = msg.battery_level >> 8;
	
		data2send[ii+10] = msg.ptx & 0xff;
		data2send[ii+11] = msg.ptx >> 8;
		
		data2send[ii+12] = msg.noise_level & 0xFF;
		data2send[ii+13] = msg.noise_level >> 8;
	
		
		data2send[ii+14] = msg.num & 0xFF;
		data2send[ii+15] = msg.num >> 8;
		
	
		
		//printf("ready to send!\n");
		uip_udp_packet_sendto(client_conn, &data2send, sizeof(data2send),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}

//START>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
/*
//serial line communication format
// START BYTE | ADDRESS | LENGTH | DATA | STOP BYTE
*/
 
static int uart_rx_callback(unsigned char c)
{
	
	 if (state == WAIT_FOR_SYNCH) {
		 if ((uint8_t)c == START_BYTE) {
			 state = 1-state; //change the state to in synch.
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
					//		printf("exceeded buffer size!!\n");
                            cnt = 0;
							
                            state = WAIT_FOR_SYNCH;
                        }
					}
			}
			 
		 }
		 else {
		//	 printf("\n%u -- %u", cnt,c);
			 rxbuf[cnt++] = c;
			 if(cnt >= SERIAL_BUF_SIZE) {
			//	 printf("exceeded buffer size!!\n");
                 cnt = 0;
                 state = WAIT_FOR_SYNCH;
             }
		}
	 }
	 return cnt;
}
//----------------------------------------------------------------------
static void biofilmdata_send(void *n)
{
	struct pck2send *e = n;
	uint8_t kk;
	 list_remove(pck2send_list, e);
	
	
	//printf("\nnow accessing packet : %d\n",e->data2send.num);
  
	uint8_t *ddmsg;
	

	ddmsg = (uint8_t *)&e->data2send;
	
	//printf("\nPacket to Send:");
	//for (kk=0;kk<sizeof(struct biofilm_datamsg);kk++) {
	//	printf("%d ",ddmsg[kk]);
	//	
	//}
	
     uip_udp_packet_sendto(client_conn, ddmsg, sizeof(struct biofilm_datamsg),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
                        
                        
     memb_free(&pck2send_memb, e);
  
	                    
	
}

uint8_t process_incoming_packet(unsigned char *incoming_packet, uint8_t size)
{
	uint8_t *tmp;
	uint8_t kk; 
	
	
		
	tmp = (uint8_t *)incoming_packet;
	
	//printf("\n Stream received from serial port:");
	//for (kk=0;kk<size;kk++) {
	//	printf("%u ", tmp[kk]);
	//	
	//}

	if (tmp[0] == 255) {
	
		
		uint8_t numofchannels = (uint8_t)((tmp[1]-1) / 6);
	//	printf("\n%i\n", numofchannels);
		//if (numofchannels == 0){
		///	return 0;
		//}
		uint8_t ss;
		if (numofchannels >= MAX_FREQS) {
			
			///split;///	
			
			
			if ((numofchannels % MAX_FREQS) == 0) 
			{
				ss = (uint8_t)(numofchannels / MAX_FREQS);
			}
			else 
			{
				ss = (uint8_t)(numofchannels / MAX_FREQS) + 1;	
			}
		}
		else {
			ss = 1;
			
		}	
		//	printf("\n%i\n", ss);
			uint8_t i, startindex, j;
			struct biofilm_datamsg new_data;
			//msn++;
			uint8_t batt_alarm = tmp[2];
			for (kk = 0; kk<ss; kk++) 
			{
			//	startindex = 2 + kk*6*MAX_FREQS;
				startindex = 3 + kk*6*MAX_FREQS;
				
				if ((kk == ss-1) && ((numofchannels % MAX_FREQS) !=0 )) {
					j = (uint8_t) numofchannels % MAX_FREQS;
				}
				else {
					if (ss > 1) {
						j = MAX_FREQS;
					}
					else {
						j = numofchannels;
					}
				}
				
				for (i=0;i<j;i++)
				{
					
					new_data.freq_index[i] = (uint8_t)tmp[startindex];// / 6;
					//printf("%d,**%d**",tmp[startindex],new_data.freq_index[i]);
					new_data.real_part[i] = (tmp[startindex+1] <<8 )| tmp[startindex+2];
					new_data.img_part[i] = (tmp[startindex+3] <<8) | tmp[startindex+4];
					new_data.temp[i] = tmp[startindex+5];
					startindex +=6;
						
					
				}
				for (i=j;i<MAX_FREQS;i++)
				{
					
					new_data.freq_index[i] = 255;
					new_data.real_part[i] = 255;
					new_data.img_part[i] = 255;
					new_data.temp[i] = 255;
					
						
					
				}
			
				new_data.totalnumofchannels = numofchannels;
		/*	printf("\nImpedance Data: ");
			for (i=0;i<MAX_FREQS;i++) {
				printf("\n%u ", new_data.freq_index[i]);
				printf("%u ", new_data.real_part[i]);
				printf("%u ", new_data.img_part[i]);
				printf("%u ", new_data.temp[i]);
		
			}*/
				new_data.msg_index = msg_index;
				new_data.battery_level = battery_sensor.value(0);
				new_data.noise_level = cc2420_get_txpower();
				new_data.ptx = (batt_alarm == 0) ?45: 46;//get_nf();
				new_data.num = msn++;
				//put into a list and let it do its job...
				update_pck2send(&new_data);

			}
			//msn++;
			uart_data_ready = 1;
		
		
	}	
	
	/**/
	return 0;
}

static void update_pck2send(struct biofilm_datamsg *data2send)
{
	
 struct pck2send *pp;
 
 /* Check if this is a duplicated packet - should not happen!*/
 
	for(pp = list_head(pck2send_list); pp != NULL; pp = list_item_next(pp)) {

		
		if(pp->data2send.num == data2send->num) 
		{
			//do nothing! - you should never be in here....
		}
	}


   if (pp == NULL)
   {
	 pp = memb_alloc(&pck2send_memb);
	 
	 if (pp == NULL)
	 {
			list_pop(pck2send_list);
			memb_free(&pck2send_memb, pp);
			
			pp = memb_alloc(&pck2send_memb);
		 
	 }

    if(pp != NULL) 
    {
	
	
    /* Initialize the fields. */
    
    memcpy(&pp->data2send, data2send,sizeof(struct biofilm_datamsg)); 
    
    /* Place the packet at the end of the list. */
    list_add(pck2send_list, pp);
    
    /*and schedule its transmission*/
    ctimer_set(&pp->ctimer, random_rand() % 20*CLOCK_SECOND, biofilmdata_send,pp);
	
	}
	
    }
  }   
	   
   	
		
	

//-----------------------------------------------------------------------
/**********************************************************************/
static void getappdata(){	
	

	
		msg.temp= sht11_sensor.value(SHT11_SENSOR_TEMP);
		msg.humm = sht11_sensor.value(SHT11_SENSOR_HUMIDITY);

		
	
		msg.battery_level = battery_sensor.value(0);
		msg.ptx = cc2420_get_txpower();
		msg.noise_level = get_nf();
     
		//----------------------------------------------------------
	
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
 
  PROCESS_BEGIN();

  PROCESS_PAUSE();

  //cc2420_set_channel(21);
  set_global_address();
  
  
 // PRINTF("UDP client process started\n");

 // print_local_addresses();

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL); 
  
  if(client_conn == NULL) {
  //  PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT)); 

 
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event && uip_newdata() && UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 2] == SINK_ID) {
   
     tcpip_handler();
    }
    
  }
  PROCESS_END();
}

/*---------------------------------------------------------------------*/
PROCESS_THREAD(sensing_process, ev,data)
{
	
	static struct etimer periodic;//, sensing_periodic;
	static struct ctimer backoff_timer;
    static uint8_t isrunning;
    static struct cmd_request_t incmd_req;
     
	static float nm_send_interval;
	static union ff tmp;
	
	uint8_t jj;

	PROCESS_BEGIN();
    SENSORS_ACTIVATE(button_sensor);
	leds_on(LEDS_RED);	
    
    isrunning = 0;
    //this is for the bf process
	send_interval = SEND_INTERVAL*600; //bf packets every 1 hour
	//and this is for the nm process
	nm_send_interval = SEND_INTERVAL * 100; //nm packets every 10mins
	
	
	standy: 
		PROCESS_WAIT_EVENT_UNTIL(ev == cmd_event || (ev == sensors_event && data == &button_sensor ));
	
	
	if (ev == cmd_event) {
	memcpy(&incmd_req, data, sizeof(struct cmd_request_t));
	//  printf("command now received: %d\n", incmd_req.cmd_index);
	if ((incmd_req.cmd_index & 0x01) == 0x01 ) //start 
	{	
			leds_off(LEDS_RED);
	
			SENSORS_ACTIVATE(sht11_sensor);
			SENSORS_ACTIVATE(battery_sensor);
			
			if ((incmd_req.cmd_index & 0x02) == 0x02) {
      				//user defined frequency
						  
					for (jj=0;jj<4;jj++){
					//consider msp430 endian format for the casting...
					tmp.b4[3-jj] = (unsigned char)incmd_req.cmd_parameter[jj];
	     
					}
					if (tmp.f <=1.0){
					//hardcoded....should not go below 1sec
						//tmp.f = 1.0;
		
						incmd_req.cmd_index |= 0x04; //an error code...
			
					}
					else {
															
						if ((incmd_req.cmd_index & 0x10) == 0x10){ // && rimeaddr_node_addr.u8[7]<=10) { //start arduino	
								
								send_interval = tmp.f; //this is for the bf sampling rate.
								
								
						}
						else {
							nm_send_interval = tmp.f;
						}
					}
				}			
				//check for arduino flag
			#ifdef BIOFILM
			if ((incmd_req.cmd_index & 0x10) == 0x10){// && rimeaddr_node_addr.u8[7]<=10) //change also the value at arduino.
						
				process_post(&biofilm_sensing_process, start_event, &incmd_req.cmd_index);
						
			}
			#endif	
				
				
				/**************UVEG MODIFICATIONS *********************/
				if(linkaddr_node_addr.u8[7]<=10) {
					etimer_set(&periodic, nm_send_interval*CLOCK_SECOND);
				}
				else {
					etimer_set(&periodic, 10*nm_send_interval*CLOCK_SECOND);
				}
				/*****************************************************/	
			
			msn=0;
			isrunning = 1-isrunning;
			//----------------------------------------------------------	
			for (jj = 0;jj<incmd_req.trace_len; jj++)
			{
				incmd_req.trace[jj] = 0;
			}
			incmd_req.trace[0] = linkaddr_node_addr.u8[7];
			incmd_req.trace_len = 1;
			incmd_req.cmd_type = (incmd_req.cmd_type == 241) ? 239 : 240;  
			ctimer_set(&backoff_timer, SEND_TIME, send_command_request, &incmd_req);
			//----------------------------------------------------------
	}
	else {
		goto standy;
		}
	}
	else
	{ //started from the user button
		
		leds_off(LEDS_RED);
	
			SENSORS_ACTIVATE(sht11_sensor);
			SENSORS_ACTIVATE(battery_sensor);
	/**************UVEG MODIFICATIONS *********************/
		if(linkaddr_node_addr.u8[7]<=10) {
			etimer_set(&periodic, nm_send_interval*CLOCK_SECOND);
		}
		else {
			etimer_set(&periodic, 10*nm_send_interval*CLOCK_SECOND);
		}
	/*****************************************************/
			
		#ifdef BIOFILM
			static uint8_t aa = 0x10; //default: sensor 1
			process_post(&biofilm_sensing_process, start_event, &aa);
		
		#endif
			
		msn=0;
		isrunning = 1-isrunning;
		
		
	}
	//------------------------------------------------------------------
	
    while (1)
    {
	  
	  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic) || ev == cmd_event);
	
	  if (ev == cmd_event){
	  memcpy(&incmd_req, data, sizeof(struct cmd_request_t));
	  
	  //start...
	  //printf("command now received: %d\n", cmd_index);
	  if ((incmd_req.cmd_index & 0x01) == 0x01 && !isrunning)
	  {
			//check for frequency update
				if ((incmd_req.cmd_index & 0x02) == 0x02) 
				{
					
					for (jj=0;jj<4;jj++){
					//consider msp430 endian format for the casting...
						tmp.b4[3-jj] = (unsigned char)incmd_req.cmd_parameter[jj];
	     
					}
					if (tmp.f <=1.0){
					//hardcoded....should not go below 1sec
						tmp.f = 1.0;
						incmd_req.cmd_index |= 0x04; //an error code...
					}
					else {
						if ((incmd_req.cmd_index & 0x10) == 0x10){// && rimeaddr_node_addr.u8[7]<=10) //change also the value at arduino.
							
							send_interval = tmp.f;
						}
						else {
							nm_send_interval = tmp.f;
									
						}
				}
				
			}
			//check for arduino flag
			#ifdef BIOFILM
			if ((incmd_req.cmd_index & 0x10) == 0x10){// && rimeaddr_node_addr.u8[7]<=10) //change also the value at arduino.
						
				process_post(&biofilm_sensing_process, start_event, &incmd_req.cmd_index);
						
			}
			#endif	
			//and then start also your timer.
			/**************UVEG MODIFICATIONS *********************/
			if(linkaddr_node_addr.u8[7]<=10) {
				etimer_set(&periodic, nm_send_interval*CLOCK_SECOND);
			}
			else {
				etimer_set(&periodic, 10*nm_send_interval*CLOCK_SECOND);
			}
			/******************************************************/			
			isrunning = 1-isrunning;				
			
	  }
      //stop
	  else if (incmd_req.cmd_index == 0 && isrunning)
	  {
			//leds_on(LEDS_RED);
				
				
			etimer_stop(&periodic);
			#ifdef BIOFILM
				process_post(&biofilm_sensing_process, start_event, &incmd_req.cmd_index);
			#endif
			isrunning = 1-isrunning;
			
	  }
	  else if ( ((incmd_req.cmd_index & 0x02) == 0x02) && isrunning) 
	  {
		  //update ts sampling period...
	//	  printf("update ts sampling period...\n");
		
		for (jj=0;jj<4;jj++){
		  //consider msp430 endian format for the casting...
	        tmp.b4[3-jj] = (unsigned char)incmd_req.cmd_parameter[jj];
	  //      printf("%u**", tmp.b4[3-jj]);
	     
		}
	//	printf("\n");
		if (tmp.f < 1.0) {
			//hardcoded....should not below 1sec
			tmp.f = 1.0;
			incmd_req.cmd_index |= 0x04; //an error code...
		//	printf("????/..\n");
		}
		else {
			if ((incmd_req.cmd_index & 0x12) == 0x12){// && rimeaddr_node_addr.u8[7]<=10) //change also the value at arduino.
							
				send_interval = tmp.f;
			}
			else {
				etimer_stop(&periodic);  
				nm_send_interval = tmp.f;
				/**************UVEG MODIFICATIONS *********************/
		
				if(linkaddr_node_addr.u8[7]<=10) {
					etimer_set(&periodic, send_interval*CLOCK_SECOND);
				}	
				else {
					etimer_set(&periodic, 10*send_interval*CLOCK_SECOND);
				}
				/*****************************************************/
									
			}
		}
		
		#ifdef BIOFILM
		if (((incmd_req.cmd_index & 0x12) == 0x12) && linkaddr_node_addr.u8[7]<=10)
		{
			process_post(&biofilm_sensing_process, start_event, &incmd_req.cmd_index);
		}
		#endif	
		  
	  }	
	  else if (((incmd_req.cmd_index & 0x30) ==  0x30) && linkaddr_node_addr.u8[7]<=10)
	  {
		  //this is an on-demand request for measurement /callibration
			#ifdef BIOFILM
				process_post(&biofilm_sensing_process, start_event, &incmd_req.cmd_index);
			#endif
		  
	  }
	  //send the command reply back to the originator...
		for (jj = 0;jj<incmd_req.trace_len; jj++){
			incmd_req.trace[jj] = 0;
		}
		incmd_req.trace[0] = linkaddr_node_addr.u8[7];
	    incmd_req.trace_len = 1;
	 
	    incmd_req.cmd_type = (incmd_req.cmd_type == 241) ? 239 : 240;
	    ctimer_set(&backoff_timer, SEND_TIME, send_command_request, &incmd_req);
	  } //done with handling the command
	  //this is the normal operation
	  else {
	  if (isrunning) {	
		
		if (etimer_expired(&periodic))
		{
			
		if (uart_data_ready == 1){	
		//	leds_toggle(LEDS_RED);
		
			getappdata();	
		
			msg.num = msn++;
			#ifdef NM
				#ifdef TRACE_ROUTE
					#ifdef TRACE_LQ
						for (jj=0;jj<MAX_NEIGHS-2;jj++){
							msg.route_link[jj] = 65535; //use this (some of the measurements may be = 0
							msg.route_nf[jj] = 255; //use this (some of the measurements may be = 0
						}
				
					#endif //TRACE_LQ
						for (jj=0;jj<2;jj++){
						msg.route_trace[jj]= 0;
					}	
	
				#endif //TRACE_ROUTE
			#endif //NM
			//for sending...
			
			ctimer_set(&backoff_timer, SEND_TIME, collect_common_send, NULL);
			}
			
			//and now reset the sampling timer
			etimer_reset(&periodic);
			}
		
	//	msn++;
	  }
	 
	 
	}
   }

	SENSORS_ACTIVATE(sht11_sensor);
	SENSORS_DEACTIVATE(battery_sensor);

	PROCESS_END();
}



PROCESS_THREAD(biofilm_sensing_process, ev, data)
{
	
	static struct etimer sensing_periodic;
	static struct etimer meas_timer;
	static uint8_t bf_running;

	static uint8_t cmdindex;
	
	//static union ff tmp1;
	PROCESS_BEGIN();
	//uart0_init(BAUD2UBR(115200)); //set the baud rate as necessary
	uart0_init(BAUD2UBR(19200)); //set the baud rate as necessary
	uart0_set_input(uart_rx_callback); //set the callback function
	
	uart_data_ready = 1;
	bf_running = 0;
	msg_index = 0;
	static uint8_t ii;
	//------------------------------------------------------------------
	//maintenance phase.
	uart0_writeb('i');
	uart0_writeb('c');//the value of the second and 3rd character should be xx
	uart0_writeb('1');
	//------------------------------------------------------------------
	
	//after maintenance, wait for an external trigger.
	PROCESS_WAIT_EVENT_UNTIL(ev == start_event);
//	printf("ready to start....\n");
	memcpy(&cmdindex, (uint8_t *)data, 1);
	leds_toggle(LEDS_BLUE);
	if ((cmdindex & 0x50) == 0x10){
		msg_index = 4;
	//	printf("\nask for calibration on resistor...\n");
		uart0_writeb('s');
		uart0_writeb('c');
	    uart0_writeb('1');
	    //msg_index = 4;	//this is the dataindex
		uart_data_ready = 0;
		sensor_mode = 0x31;//sensor 1 --callibration
	}
	else if ((cmdindex & 0x50) == 0x50) {
		msg_index = 4;
	//	printf("\nask for calibration on resistor...\n");
		uart0_writeb('s');
		uart0_writeb('c');
	  //  uart0_writeb('2');
		uart0_writeb('1');
	  //  msg_index = 5;
//		msg_index = 4;	//this is the dataindex			
		uart_data_ready = 0;
		sensor_mode = 0x32; //sensor 2 -- callibration	
	}
	process_post(&uart_connection_error_handling, tx_cmd_event, NULL);				
	//and now N successive measurements...
	
	for (ii=0;ii<NNN;ii++){
		etimer_set(&meas_timer, 120*CLOCK_SECOND);
		
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&meas_timer));
		msg_index = (sensor_mode == 0x31) ? 6 : 7;
	//	printf("\nask for measurement from sensor %i...\n",sensor_mode);
		uart0_writeb('s');
		uart0_writeb('m');
		uart0_writeb(sensor_mode);
		uart_data_ready = 0;
		process_post(&uart_connection_error_handling, tx_cmd_event, NULL);	
	}
	
	 leds_toggle(LEDS_BLUE);	
	//printf("done with callibration!\n");
	
	
	etimer_set(&sensing_periodic, send_interval*CLOCK_SECOND);
	bf_running = 1;
	
		
	
	
	while (1) 
	{
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sensing_periodic) || ev == start_event);
		
		if (ev == start_event)
		{
			
		
			memcpy(&cmdindex, (uint8_t *)data, 1);
		
			if (cmdindex  == 0)
			{
			//		printf("\nstopping...\n");
					if (bf_running == 1) 
					{
	
					//stop
						etimer_stop(&sensing_periodic);
						etimer_stop(&meas_timer);
						bf_running = 1-bf_running;
						
						//----------------------------------------------
						//maintenance phase.
						uart0_writeb('i');
						uart0_writeb('c');//the value of the second and 3rd character should be xx
						uart0_writeb('1');
						//----------------------------------------------
					}
			
				
			}
			
			else if (cmdindex == 0x10 && bf_running) //recallibration using sensor 1
			{
				etimer_stop(&sensing_periodic);
				etimer_stop(&meas_timer);
			//	printf("\nask for calibration on resistor...\n");
				msg_index = 4;
				uart0_writeb('s');
				uart0_writeb('c');
				uart0_writeb('1');			
				uart_data_ready = 0;
				sensor_mode = 0x31;
			    process_post(&uart_connection_error_handling, tx_cmd_event, NULL);	
				
				for (ii=0;ii<NNN;ii++){
					etimer_set(&meas_timer, 120*CLOCK_SECOND);
					PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&meas_timer));
				//	printf("\nask for measurement from sensor %i...\n",sensor_mode);
					msg_index = 6;
					uart0_writeb('s');
					uart0_writeb('m');
					uart0_writeb(sensor_mode);
					uart_data_ready = 0;
					process_post(&uart_connection_error_handling, tx_cmd_event, NULL);	
				}

				etimer_set(&sensing_periodic, send_interval*CLOCK_SECOND);
			}
			
			else if (cmdindex == 0x50 && bf_running){ //recallibration using sensor 2
				etimer_stop(&sensing_periodic);
				etimer_stop(&meas_timer);
		//		printf("\nask for calibration on resistor...\n");
				msg_index = 4;
				
				uart0_writeb('s');
				uart0_writeb('c');
				//uart0_writeb('2');		
				uart0_writeb('1');		
				uart_data_ready = 0;
				
				sensor_mode = 0x32;
				process_post(&uart_connection_error_handling, tx_cmd_event, NULL);	
				
				for (ii=0;ii<NNN;ii++){
					etimer_set(&meas_timer, 120*CLOCK_SECOND);
					PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&meas_timer));
			//		printf("\nask for measurement from sensor %i...\n",sensor_mode);
					msg_index = 7;
					uart0_writeb('s');
					uart0_writeb('m');
					uart0_writeb(sensor_mode);
					uart_data_ready = 0;
					process_post(&uart_connection_error_handling, tx_cmd_event, NULL);	
				}
				etimer_set(&sensing_periodic, send_interval*CLOCK_SECOND);
				
			}
			else if (((cmdindex & 0x11) == 0x11) && !bf_running){
			//	printf("**%d**\n", cmdindex);
				if ((cmdindex &0x50) == 0x10){
			//		printf("\nask for calibration on resistor...\n");
					msg_index = 4;
					uart0_writeb('s');
					uart0_writeb('c');
					uart0_writeb('1');			
					uart_data_ready = 0;
					sensor_mode = 0x31;
						
					
				}
				else if ((cmdindex & 0x50) == 0x50){
					msg_index = 4;
					//printf("sensor2\n");
				//	printf("\nask for calibration on resistor...\n");
					uart0_writeb('s');
					uart0_writeb('c');
				//uart0_writeb('2');
				uart0_writeb('1');			
				uart_data_ready = 0;
				sensor_mode = 0x32;
				}
				process_post(&uart_connection_error_handling, tx_cmd_event, NULL);	
				
				bf_running = 1-bf_running;
				//msg_index = (sensor_mode == 1) ? 6 : 7;
				for (ii=0;ii<NNN;ii++){
					etimer_set(&meas_timer, 120*CLOCK_SECOND);
					PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&meas_timer));
				//	printf("\nask for measurement from sensor %i...\n",sensor_mode);
					msg_index = (sensor_mode == 0x31) ? 6 : 7;
					uart0_writeb('s');
					uart0_writeb('m');
					uart0_writeb(sensor_mode);
					uart_data_ready = 0;
					process_post(&uart_connection_error_handling, tx_cmd_event, NULL);	
				}
	
				
				
				etimer_set(&sensing_periodic, send_interval*CLOCK_SECOND);
				
			}
			
			else if (cmdindex  == 0x30){
						etimer_stop(&sensing_periodic);
						etimer_stop(&meas_timer);
						//ask for a measurement from sensor 1
						msg_index = 4;
				//		printf("\nask for calibration on resistor...\n");
						uart0_writeb('s');
						uart0_writeb('c');
						uart0_writeb('1');			
						uart_data_ready = 0;
						process_post(&uart_connection_error_handling, tx_cmd_event, NULL);	
						for (ii=0;ii<NNN;ii++){
							etimer_set(&meas_timer, 120*CLOCK_SECOND);
							PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&meas_timer));
						//	printf("\nask for measurement from sensor %i...\n",sensor_mode);
							msg_index = 6;	
							uart0_writeb('s');
							uart0_writeb('m');
							uart0_writeb('1');			
							uart_data_ready = 0;	
							process_post(&uart_connection_error_handling, tx_cmd_event, NULL);	
						}
						etimer_set(&sensing_periodic, send_interval*CLOCK_SECOND);	
			}
			else if (cmdindex == 0x70){
					etimer_stop(&sensing_periodic);
						etimer_stop(&meas_timer);
				msg_index = 4;
				//ask for a measurement from sensor 2		
		//		printf("\nask for calibration on resistor...\n");
				uart0_writeb('s');
				uart0_writeb('c');
				uart0_writeb('1');			
				uart_data_ready = 0;
				process_post(&uart_connection_error_handling, tx_cmd_event, NULL);	
				for (ii=0;ii<NNN;ii++){
					etimer_set(&meas_timer, 120*CLOCK_SECOND);
					PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&meas_timer));
				//	printf("\nask for measurement from sensor %i...\n",sensor_mode);
					msg_index = 7;	
					uart0_writeb('s');
					uart0_writeb('m');
					uart0_writeb('2');			
					uart_data_ready = 0;
					process_post(&uart_connection_error_handling, tx_cmd_event, NULL);		
				}
				etimer_set(&sensing_periodic, send_interval*CLOCK_SECOND);	
			}
			
			else if (cmdindex == 0x12 && bf_running) //reset timer 
			{
					
		//		printf("\nreseting timer...\n");
				etimer_stop(&sensing_periodic);
				etimer_set(&sensing_periodic, send_interval*CLOCK_SECOND);
				
			}	
					
		}
		
	
		else
		{
			
			if (bf_running == 1){
			//  printf("ready...\n");
			//0. recalibrate
				msg_index = 4;
				//ask for a measurement from sensor 2		
			//	printf("\nask for calibration on resistor...\n");
				uart0_writeb('s');
				uart0_writeb('c');
				uart0_writeb('1');			
				uart_data_ready = 0;
				process_post(&uart_connection_error_handling, tx_cmd_event, NULL);	
				
			//1. ask for data from arduino - the rest are taken care of by process_incoming_packet
				for (ii=0;ii<NNN;ii++){
					etimer_set(&meas_timer, 120*CLOCK_SECOND);
					PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&meas_timer));
			//		printf("\nask for measurement from sensor %i...\n",sensor_mode);
					msg_index = (sensor_mode == 0x31) ? 6 : 7;
					uart0_writeb('s');
					uart0_writeb('m');
					uart0_writeb((char)sensor_mode);
					uart_data_ready = 0;
					process_post(&uart_connection_error_handling, tx_cmd_event, NULL);	
				}
		  
			etimer_reset(&sensing_periodic);
			}
		}
	}
	PROCESS_END();
}

/*---------------------------------------------------------------------*/
PROCESS_THREAD(c2c_process, ev, data)
{
   
  static struct cmd_request_t appdata;
  static struct ctimer ctimer;	
  
 
  PROCESS_BEGIN();
  PROCESS_PAUSE();
  
  
  c2c_conn = udp_new(NULL,UIP_HTONS(UDP_MOUT_PORT), NULL); 
 
  if(c2c_conn == NULL) {
 
    PROCESS_EXIT();
  }
  udp_bind(c2c_conn, UIP_HTONS(UDP_MOUT_PORT)); 
  previous_msn = 0xffff;
 
 
  while (1) {
	  
	  PROCESS_YIELD();
	  
		if (ev == tcpip_event && uip_newdata() && UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 2] !=SINK_ID){
		
			tcpip_handler();
	
		}
		
		if (ev == event_data_ready){
			
//			printf("received a command from downlink connection!\n");
			memcpy(&appdata, (struct cmd_request_t *)data, sizeof(struct cmd_request_t));
			ctimer_set(&ctimer, random_rand()%(11*CLOCK_SECOND), send_command_request, &appdata);
			
		}
	}
	  
    
  PROCESS_END();
}
////////////////////////////////////////////////////////////////////////
PROCESS_THREAD(uart_connection_error_handling,ev,data){
	
	static struct etimer rxtimer;
	static uint8_t rxtimeout;
	
	 static struct cmd_request_t error_not;
	PROCESS_BEGIN();
	
	rxtimeout = 0;
	
	while (1)
	{
		PROCESS_YIELD();
		
		if (ev == tx_cmd_event)
		{
		
			//else
			//{
				//leds_off(LEDS_ALL); // for now	
				etimer_set(&rxtimer, 90*CLOCK_SECOND);
				PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&rxtimer));
				if (uart_data_ready == 0) // you are still receiving
				{
					if (state == IN_SYNCH){
					//you are blocked in uart rx
					//1. stop rx 
					  printf("\nblocked in rx - reseting rx buffer\n");
					  state = WAIT_FOR_SYNCH;
					  cnt =0;
					}
					else {//else you never received anything
						printf("\nuart connection timed out without a response\n");
					}
					
					//2. keep a counter of successive failures
					rxtimeout++;
					//3. unblock the nm packets
					uart_data_ready = 1;
					
				
				}
				else {//ok....
					rxtimeout = 0;
					leds_off(LEDS_ALL); // for now	
				}
				
				if (rxtimeout >= 4) //4: 2 successive measurement cycles.
				{
				//	printf("\npermanent uart connection error\n");
				//	leds_on(LEDS_ALL); // for now	
				//send a error notification packet to mServer / Gateway. 
				//error code: 255
					error_not.cmd_index = 0xff;
					error_not.cmd_type = 240; //for gateway
					error_not.htl = 0;
					error_not.trace[0] = linkaddr_node_addr.u8[7];
					error_not.trace_len = 1;
	 
	    
			//	ctimer_set(&backoff_timer, SEND_TIME, send_command_request, &error_not);
					send_command_request(&error_not);
					rxtimeout = 0;
				}
			
		}
		
	}

	PROCESS_END();
	
	
	}
