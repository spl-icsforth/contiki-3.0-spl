/*
 * Copyright (c) 2013, Institute of Computer Science, FORTH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * 
 */

#include "contiki.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/ip/uip.h"
#include "dev/leds.h"


#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "lib/list.h"
#include "lib/memb.h"


#include "nm-common.h"
#ifndef PERIOD
#define PERIOD 5
#endif

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//2014-05-08 nancy @ FORTH-ICS
/*
#ifdef WITH_DL && NM
//2014-05-08 nancy @FORTH-ICS
/* This structure holds information about our children. 
struct children {
  /* The ->next pointer is needed since we are placing these on a
Contiki list. 
  struct children   *next;

  /* The ->addr field holds the LL address of the child. 
  uip_lladdr_t addr;
  
 // rimeaddr_t rimeaddr;
  
  //the time out timer for removing old entries
  struct ctimer ctimer;

};
*/
/* This #define defines the maximum amount of neighbors we can remember. */
//#define CHILDREN_TIMEOUT 10*PERIOD*CLOCK_SECOND

/* This MEMB() definition defines a memory pool from which we allocate
neighbor entries. */
//MEMB(children_memb, struct children, MAX_CHILDREN);

/* The neighbors_list is a Contiki list that holds the neighbors we
have seen thus far. */
//LIST(children_list);

//static void update_children(void);
//static void remove_child(void *n);


//#endif

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
static uint8_t get_meannf();


//2013-11-27 nancy @ FORTH-ICS

#define MAX_NF 5
static uint8_t cnt;
static uint8_t nf[MAX_NF];
static uint8_t cnttot;

/*---------------------------------------------------------------------------*/
PROCESS(nm_common_process, "nm common process");
/*---------------------------------------------------------------------------*/

#ifdef TRACE_LQ
void update_uip6pck(int nwk_header, int rssi_lqi)
#else
void update_uip6pck(int nwk_header)
#endif
	{
	  
	   

	int i,j;
	int route_flag;
	
	
//	if (UIP_IP_BUF->proto == UIP_PROTO_UDP) - in contrast to ipv4 (uip-over-rime)
//	{
	
		#ifdef TRACE_LQ
			uint16_t route_link[MAX_NEIGHS-2];
			uint8_t route_nf[MAX_NEIGHS-2];
			
		#endif
	
		uint16_t trace_route[TRACEROUTE_LEN];
	
	
	j=0;
	//retrieve
	#ifdef TRACE_LQ
	for (i=0;i<(MAX_NEIGHS-2)*2;i=i+2)
	{
		route_link[j] = uip_buf[UIP_IPUDPH_LEN+i+nwk_header+1] <<8 | uip_buf[UIP_IPUDPH_LEN+i+nwk_header];
		j++;
	}
	
	j=0;
	for (i=(MAX_NEIGHS-2)*2;i<(MAX_NEIGHS-2)*3;i++)
	{
		route_nf[j] = uip_buf[UIP_IPUDPH_LEN+i+nwk_header];
		j++;
	}
	
	j=0;
	for (i=(MAX_NEIGHS-2)*3;i<(MAX_NEIGHS-2)*3+TRACEROUTE_LEN*2;i=i+2)
	{
		trace_route[j] = uip_buf[UIP_IPUDPH_LEN+i+nwk_header] <<8 | uip_buf[UIP_IPUDPH_LEN+i+1+nwk_header];
		j++;
	}
	
	j=0;
	#else
	for (i=0;i<TRACEROUTE_LEN*2;i=i+2)
	{
		trace_route[j] = uip_buf[UIP_IPUDPH_LEN+i+nwk_header] <<8 | uip_buf[UIP_IPUDPH_LEN+i+1+nwk_header];
		//printf("**%d**",trace_route[j]);
		j++;
	}
	#endif //TRACE_LQ
	i=0;
	route_flag = 0;
 
 
 	
    //and update
	#ifdef TRACE_LQ
		while (i<(MAX_NEIGHS-2))
		{
			if (route_link[i] == 0xffff){
				route_link[i] = rssi_lqi;

				break;
			}
			i++;
		}
	
		
		i=0;
		while (i<(MAX_NEIGHS-2))
		{
			
		
			if (route_nf[i] == 0xff){
				
				route_nf[i] = get_meannf();
				
				break;
			}
			i++;
		}
	
		i=0;

		#endif //TRACE_QLINK
	
		while (i<TRACEROUTE_LEN)	{
			if (trace_route[i] < 1111){
			//    printf("about to add to route: %d - to route:%d\n", uip_lladdr.addr[5], trace_route[i]);	
				route_flag = 1;
				break;
			}
		i++; 
	}	
	//using the link layer (ll) address of the node (6 bytes long) - partially included in the ipv6 address.
	//the ll of the packet's origin is in the rpl nwk header 
	if (route_flag){
		if (trace_route[i] ==0){
		
			trace_route[i] |= uip_lladdr.addr[5];
		}
		else{
			int tmp,nn, j=0;
			nn=2;
			tmp = trace_route[i]>>4;
			while (tmp!=0 && j<2)  {
				if (j==0){
					nn = 4;
				}
				if (j==1){
					nn = 8;
				}
				
				j++;	
				tmp = tmp>>4;
								
			} 
			
			if (tmp == 0){
				if (nn == 2){
					trace_route[i] |= (uip_lladdr.addr[5]<<4);
				}
				if (nn == 4){
					
					trace_route[i] |= (uip_lladdr.addr[5]<<8);				
					
				}
				if (nn== 8){
					trace_route[i] |= (uip_lladdr.addr[5]<<12);
					
				}
				
			}	
		}
		//printf("added to route: %d - now route is:%d\n", uip_lladdr.addr[5], trace_route[i]);
		//put it back to the uip_buf
		j= 0;
		#ifdef TRACE_LQ
		for (i=UIP_IPUDPH_LEN+nwk_header;i<UIP_IPUDPH_LEN+nwk_header+(MAX_NEIGHS-2)*2;i=i+2) {
			uip_buf[i] = route_link[j] & 0xff; //lsb
			uip_buf[i+1] = route_link[j] >> 8; //msb
			j++;
			
	    }
	    
		j=0;
		for (i=UIP_IPUDPH_LEN+nwk_header+(MAX_NEIGHS-2)*2;i<UIP_IPUDPH_LEN+nwk_header+(MAX_NEIGHS-2)*3;i++) {
			uip_buf[i] = route_nf[j];
			
			j++;
			
	    }
		
		j=0;
		for (i=UIP_IPUDPH_LEN+nwk_header+(MAX_NEIGHS-2)*3;i<UIP_IPUDPH_LEN+nwk_header+TRACEROUTE_LEN*2+(MAX_NEIGHS-2)*3;i=i+2) {
			uip_buf[i] = trace_route[j] >> 8; //msb
			uip_buf[i+1] = trace_route[j] & 0xff; //lsb
			j++;
			
	    }
		#else
		for (i=UIP_IPUDPH_LEN+nwk_header;i<UIP_IPUDPH_LEN+nwk_header+TRACEROUTE_LEN*2;i=i+2) {
			uip_buf[i] = trace_route[j] >> 8; //msb
			uip_buf[i+1] = trace_route[j] & 0xff; //lsb
			j++;
			
	    }
	    #endif //TRACE_LQ
//	}
	
 
}

//---------------------------	 
	
}
//#ifdef TRACE_LQ
static uint8_t get_meannf()
{
	
	int j,k;
	int tmp = 0;
	
	if (cnttot >= MAX_NF) {
		k = MAX_NF;
	}
	else{
		k = cnttot;
	}	
	
		for (j=0;j<k;j++)
		{
			tmp += nf[j];
	   	
		
		}
  	
	//	printf("%i,%i,%i***%i%%%%%i\n", cnttot,k,cnt, tmp, tmp/j);
		if (j>0) {
			tmp = (int) tmp / j;
			return tmp & 0xff;
		
		}
		else {
			return 0xff;
				
		}
	
		
	
	
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//for the upper layer...
uint8_t get_nf(void){
	return get_meannf();
	}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<	

#ifdef NM
void nm_input()
{
	
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//2014-05-08

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
if (linkaddr_node_addr.u8[7] != SINK_ID){


//not applicable for biofilm packets....
//printf("%d, %d, %d, %d\n", uip_len,UIP_IPUDPH_LEN,uip_datalen(), uip_ext_len);
//this is the application payload size. The overhead of the rest of the layers is 48 bytes
	if (uip_len - (UIP_IPUDPH_LEN + uip_ext_len) > 65) 
	{
			
		//	printf("this is a longgggg packet.....\n");
			 
			return;
		
	}
#ifndef TRACE_LQ
			update_uip6pck(uip_ext_len);
	#else
			uint8_t rssival = packetbuf_attr(PACKETBUF_ATTR_RSSI) & 0xff; 
			uint8_t lqival = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY) & 0xff; 
			int rssilqival = (rssival <<8) | lqival ;
			
			update_uip6pck(uip_ext_len, rssilqival);
	#endif	
}

/*else {
	#ifdef WITH_DL


		update_children();


	#endif
	
	
	}	*/

}


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//2014-05-08
//nancy @ FORTH-ICS
/*
 #ifdef WITH_DL
 static void update_children(void)
 {
	struct children *n;
	uip_lladdr_t *tmp;  
	  
	tmp = (uip_lladdr_t *)packetbuf_addr(PACKETBUF_ADDR_SENDER);
	
	/* Check if we already know this child. 
	for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {

		/* We break out of the loop if the LSB of the ll address of the child matches
		the address of the child from which we received this IPv6 msg 
		
		if(n->addr.addr[5] == tmp->addr[5]) {
			 /* Our child was found, so we update the timeout. 
			 
			ctimer_set(&n->ctimer, CHILDREN_TIMEOUT, remove_child, n);
			break;
		}
	}

  /* If n is NULL, this child was not found in our list, and we
allocate a new struct child from the children_memb memory
pool. 
  if(n == NULL) {
    n = memb_alloc(&children_memb);

    /* If we could not allocate a new children entry, we give up. We
could have reused an old neighbor entry, but we do not do this
for now. 
    if(n != NULL) {
    /* Initialize the fields. 
    //rimeaddr_copy(&n->rimeaddr, packetbuf_addr(PACKETBUF_ADDR_SENDER));
    memcpy(&n->addr, (uip_lladdr_t *)packetbuf_addr(PACKETBUF_ADDR_SENDER),sizeof(uip_lladdr_t)); 
    //n->addr = (uip_lladdr_t )packetbuf_addr(PACKETBUF_ADDR_SENDER);
    /* Place the child on the children list at the end of the list. 
    list_add(children_list, n);
    
    
    ctimer_set(&n->ctimer, CHILDREN_TIMEOUT, remove_child, n);
	
    
   // printf("My children are: ");
   // for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
	// printf("%d.%d.%d.%d.%d.%d, ", n->addr.addr[0],n->addr.addr[1],n->addr.addr[2],n->addr.addr[3],
	// n->addr.addr[4],n->addr.addr[5]);
		
		
	//}
	//printf("\n");
    //printf("child %d added on my list!\n", n->addr.u8[7]);
    }
   }
   
}

/*
 * This function is called by the ctimer present in each neighbor
 * table entry. The function removes the neighbor from the table
 * because it has become too old.
 
static void remove_child(void *n)
{
  struct children *e = n;
 //removing old items...
 //printf("now removing node: %d\n",e->addr.addr[5]);
  list_remove(children_list, e);
  memb_free(&children_memb, e);
  
}

uint8_t get_nmlist_length()
{
	return list_length(children_list);
}

uint8_t get_nmlist(uip_lladdr_t *listaddr, uint8_t size)
{
   
   struct children *n;
   uip_lladdr_t tmp;
   
   uint8_t i;
   if (size >0){
	   
   i=0;
   for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
	memcpy(&tmp, &n->addr, sizeof(uip_lladdr_t));
	listaddr[i++] = tmp;
   }
   }
   else {
	   
	listaddr = NULL;   
   }	
   return i;
	
}
#endif /*WITH_DL*/
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

#endif /*NM*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(nm_common_process, ev, data)
{
  static struct etimer period_timer;
 
  PROCESS_BEGIN();

   
  etimer_set(&period_timer, CLOCK_SECOND * PERIOD);
  cnt=0;
  cnttot = 0;
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&period_timer));
     
     //nf[cnt] = NETSTACK_RADIO.rssi();
     nf[cnt] = cc2420_last_rssi();
     cnt=(cnt+1) % MAX_NF;     
	 cnttot = (cnttot+1)% 255;
     
    
     
	//calc mean rssi
     etimer_reset(&period_timer);
    
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/



