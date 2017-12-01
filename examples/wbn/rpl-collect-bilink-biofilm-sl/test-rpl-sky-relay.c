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
#include "net/linkaddr.h"
//#include "net/rime.h"
//#include "net/netstack.h"

//corrections here
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
//#include "net/neighbor-info.h"
#include "net/rpl/rpl.h"
#include "nm-common.h"


#include <stdio.h>
#include <string.h>
//#include <math.h>

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

#define UDP_MOUT_PORT 1234




#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define CMD_TIMER 5*CLOCK_SECOND

#define SEND_INTERVAL	6
#define SEND_TIME		(random_rand() % (6*SEND_INTERVAL))


//#define DEBUG DEBUG_PRINT
//#include "net/uip-debug.h"

static struct uip_udp_conn *client_conn;
static struct uip_udp_conn *c2c_conn;

//static struct unicast_conn unicast;

static uip_ipaddr_t server_ipaddr;
static uint16_t previous_msn;


static process_event_t event_data_ready, cmd_event;


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
//PROCESS(c2c_relay_process, "c2c relay process");


#ifdef NM
 #ifdef TRACE_ROUTE
	#ifdef TRACE_LQ
		AUTOSTART_PROCESSES(&udp_client_process, &sensing_process, &nm_common_process,&c2c_process);//,&c2c_relay_process);
	#endif	
 #endif
 #else
	AUTOSTART_PROCESSES(&udp_client_process, &sensing_process, &c2c_process);//,&c2c_relay_process);
#endif

/**********************************************************************/
static void
tcpip_handler(void)
{
  if(uip_newdata()) {
	  
	
	  //if a receive a command_request with an msn that i have replied to before or with hop counter == 0, ignore 
    /* Ignore incoming data */
    //else 
   
    uint8_t i;
     uint8_t *cmd_type;
	struct cmd_request_t *appdata2;
//	struct ctimer ctimer;
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */ 
  
    cmd_type = (uint8_t *)uip_appdata;
    if (cmd_type[0] >240 && uip_datalen() == sizeof(struct cmd_request_t))
    {
		//memcpy(&appdata,(struct cmd_request_t *)uip_appdata, sizeof(struct cmd_request_t));
		appdata2 = (struct cmd_request_t *)uip_appdata;
	//	printf("*********%d,%d:", previous_msn, appdata2->htl);
		if (appdata2->htl>=0 && appdata2->cmd_msn != previous_msn) {
	   
		//1. read command and process command
		//note: for this version: start and stop data collection
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
	//this is a broadcast packet for everybody in my range. ( a relaying of a command request)	
		uip_create_linklocal_allnodes_mcast(&c2c_conn->ripaddr);
		
		uip_udp_packet_send(c2c_conn, msg, sizeof(struct cmd_request_t));
		uip_create_unspecified(&c2c_conn->ripaddr);
	}
	
	else if (cc->cmd_type == 240 || cc->cmd_type == 239){
		//this is a reply just for the mserver
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


/**********************************************************************/
static void getappdata(){	
	
	

		msg.temp= sht11_sensor.value(SHT11_SENSOR_TEMP);
		msg.humm = sht11_sensor.value(SHT11_SENSOR_HUMIDITY);
	
		msg.battery_level = battery_sensor.value(0);
		msg.ptx = cc2420_get_txpower();
		msg.noise_level = get_nf();

	
	
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
 
  static struct ctimer ctimer;	
  
  PROCESS_BEGIN();

  PROCESS_PAUSE();

  set_global_address();
  
 
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
     //ctimer_set(&ctimer, random_rand()*CLOCK_SECOND,tcpip_handler, NULL);//();
     tcpip_handler();
    }
    
  }
  PROCESS_END();
}

/*---------------------------------------------------------------------*/
PROCESS_THREAD(sensing_process, ev,data)
{
	
	static struct etimer periodic;
	static struct ctimer backoff_timer;
    static uint8_t isrunning;
    static struct cmd_request_t incmd_req;
	static uint16_t i;
	static union ff tmp;
	static float send_interval;
	uint8_t jj;

	PROCESS_BEGIN();

	SENSORS_ACTIVATE(button_sensor);

 	leds_on(LEDS_RED);	
    
	send_interval = SEND_INTERVAL*1; //initial value: 10 minutes
	isrunning = 0;
	
	standby: 
		//PROCESS_WAIT_EVENT_UNTIL(ev == cmd_event || (ev == sensors_event && data == &button_sensor ));
	if (ev == cmd_event){	
		memcpy(&incmd_req, data, sizeof(struct cmd_request_t));
		//  printf("command now received: %d\n", incmd_req.cmd_index);
		//if (incmd_req.cmd_index ==1 )
		if ((incmd_req.cmd_index & 0x01) == 0x01 )
		{
		
			leds_off(LEDS_RED);
			//for memory space - to be removed when xm1000 are employed as relays...
		//	if(rimeaddr_node_addr.u8[7]<=10) {
			SENSORS_ACTIVATE(light_sensor);
			SENSORS_ACTIVATE(sht11_sensor);
		//	}
			SENSORS_ACTIVATE(battery_sensor);
		
				
			/**************UVEG MODIFICATIONS *********************/
			etimer_set(&periodic, send_interval*CLOCK_SECOND); //*3 = 30minutes
			/*****************************************************/
	
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
			i=0;
			
		}
		else {
			goto standby;
		}	
	}
	else {
		leds_off(LEDS_RED);
	
		SENSORS_ACTIVATE(sht11_sensor);
		SENSORS_ACTIVATE(battery_sensor);
	/**************UVEG MODIFICATIONS *********************/
		
		etimer_set(&periodic, send_interval*CLOCK_SECOND);
		
	/*****************************************************/
			
	
			
	//	msn=0;
		isrunning = 1-isrunning;
	}
	i=0;
    while (1)
    {
	  
	  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic) || ev == cmd_event);
	  if (ev == cmd_event){
	  memcpy(&incmd_req, data, sizeof(struct cmd_request_t));
	 
	  if ((incmd_req.cmd_index & 0x01) == 0x01 && !isrunning)
	  {
		
				
		
				/**************UVEG MODIFICATIONS *********************/
				//relays
				etimer_set(&periodic, send_interval*CLOCK_SECOND);
						
				isrunning = 1-isrunning;
		
	  }
      //stop
	  else if (incmd_req.cmd_index == 0 && isrunning)
	  {
				
				etimer_stop(&periodic);
				isrunning = 1-isrunning;
		
	  }
	  else if (((incmd_req.cmd_index & 0x02) == 0x02) && ((incmd_req.cmd_index & 0x10) == 0)) 
	  {
			  //update ts sampling period...
		//  printf("update ts sampling period...\n")
		etimer_stop(&periodic);  
		for (jj=0;jj<4;jj++){
		  //consider msp430 endian format for the casting...
	        tmp.b4[3-jj] = (unsigned char)incmd_req.cmd_parameter[jj];
	      //  printf("%u**", tmp.b4[3-jj]);
	     
		}
		//printf("\n");
		if (tmp.f < 1.0) {
			//hardcoded....should not below 1sec
			//tmp.f = 1.0;
			incmd_req.cmd_index |= 0x04; //an error code...
			//printf("????/..\n");
		}
		else {
			send_interval = tmp.f;
		}
		/**************UVEG MODIFICATIONS *********************/
		
      	
	    etimer_set(&periodic, 3*send_interval*CLOCK_SECOND);
		 
	  }	
	  //send the command reply back to the originator...
		for (jj = 0;jj<incmd_req.trace_len; jj++){
				incmd_req.trace[jj] = 0;
			}
			incmd_req.trace[0] = linkaddr_node_addr.u8[7];
	        incmd_req.trace_len = 1;
	        //incmd_req.cmd_type = 240;
	        //incmd_req.cmd_index = isrunning;
	        incmd_req.cmd_type = (incmd_req.cmd_type == 241) ? 239 : 240;
	        ctimer_set(&backoff_timer, SEND_TIME, send_command_request, &incmd_req);
	  }
	  else {
	  if (isrunning) {	
		
		
		getappdata();
		
		msg.num = i++;
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
		//and now reset the sampling timer
		etimer_reset(&periodic);
		
	  }
	 
	 
	}
	}
	//for memory space - to be removed when xm1000 are employed as relays...
	//		if(rimeaddr_node_addr.u8[7]<=10) {
	SENSORS_DEACTIVATE(light_sensor);
	SENSORS_DEACTIVATE(sht11_sensor);
//}
	SENSORS_DEACTIVATE(battery_sensor);

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
  //client_conn = udp_new(&ipaddr, UIP_HTONS(UDP_SERVER_PORT), NULL); 
  if(c2c_conn == NULL) {
 //   PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(c2c_conn, UIP_HTONS(UDP_MOUT_PORT)); 
  previous_msn = 0xffff;
  
 // PRINTF("Created a connection with the server ");
 // PRINT6ADDR(&c2c_conn->ripaddr);
 // PRINTF(" local/remote port %u/%u\n",
//        UIP_HTONS(c2c_conn->lport), UIP_HTONS(c2c_conn->rport));
 
  while (1) {
	  
	  PROCESS_YIELD();
	  
		if (ev == tcpip_event && uip_newdata() && UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 2] !=SINK_ID){
	//	leds_toggle(LEDS_GREEN);
	//	printf("received smth from p2p connection!\n");
		tcpip_handler();
	//	ctimer_set(&ctimer1, random_rand()%CLOCK_SECOND,tcpip_handler, NULL);
		
		}
		
		if (ev == event_data_ready){
			memcpy(&appdata, (struct cmd_request_t *)data, sizeof(struct cmd_request_t));
			ctimer_set(&ctimer, random_rand()%(10*CLOCK_SECOND), send_command_request, &appdata);
			
			}
	
			
			
	}
	  
    
  PROCESS_END();
}

