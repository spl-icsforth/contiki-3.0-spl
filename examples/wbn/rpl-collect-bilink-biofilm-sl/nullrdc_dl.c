/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 *
 * $Id: nullrdc.c,v 1.4 2010/11/23 18:11:00 nifi Exp $
 */

/**
 * \file
 *         A null RDC implementation that uses framer for headers.
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Niclas Finne <nfi@sics.se>
 */
#include "contiki.h"
 
 #include "lib/random.h"

#include "net/mac/nullrdc.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include <string.h>

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifdef NULLRDC_CONF_ADDRESS_FILTER
#define NULLRDC_ADDRESS_FILTER NULLRDC_CONF_ADDRESS_FILTER
#else
#define NULLRDC_ADDRESS_FILTER 1
#endif /* NULLRDC_CONF_ADDRESS_FILTER */

#ifndef NULLRDC_802154_AUTOACK
#ifdef NULLRDC_CONF_802154_AUTOACK
#define NULLRDC_802154_AUTOACK NULLRDC_CONF_802154_AUTOACK
#else
#define NULLRDC_802154_AUTOACK 0
#endif /* NULLRDC_CONF_802154_AUTOACK */
#endif /* NULLRDC_802154_AUTOACK */

#ifndef NULLRDC_802154_AUTOACK_HW
#ifdef NULLRDC_CONF_802154_AUTOACK_HW
#define NULLRDC_802154_AUTOACK_HW NULLRDC_CONF_802154_AUTOACK_HW
#else
#define NULLRDC_802154_AUTOACK_HW 0
#endif /* NULLRDC_CONF_802154_AUTOACK_HW */
#endif /* NULLRDC_802154_AUTOACK_HW */

#if NULLRDC_802154_AUTOACK
#include "sys/rtimer.h"
#include "dev/watchdog.h"

#define ACK_WAIT_TIME                      RTIMER_SECOND / 2500
#define AFTER_ACK_DETECTED_WAIT_TIME       RTIMER_SECOND / 1500
#define ACK_LEN 3
#endif /* NULLRDC_802154_AUTOACK */

#if NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW
struct seqno {
  rimeaddr_t sender;
  uint8_t seqno;
};

#ifdef NETSTACK_CONF_MAC_SEQNO_HISTORY
#define MAX_SEQNOS NETSTACK_CONF_MAC_SEQNO_HISTORY
#else /* NETSTACK_CONF_MAC_SEQNO_HISTORY */
#define MAX_SEQNOS 16
#endif /* NETSTACK_CONF_MAC_SEQNO_HISTORY */

static struct seqno received_seqnos[MAX_SEQNOS];
#endif /* NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW */


/***********************FORTH Modifications**************************/
#ifdef NM
#include "rdc.h"
#include "lib/list.h"
#include "lib/memb.h"
#ifndef PERIOD
#define PERIOD 5
#endif
void set_tx_counter(uint8_t tx_counter);
void set_rx_counter();
void get_mac_counters();
static struct mac_counters mac_txrx_counter;
static struct mac_counters get_txrx_counters(void);

#ifdef RIMEADDR_CONF_SIZE //Rime case
#define RIMEADDR_SIZE_RDC RIMEADDR_CONF_SIZE
#else
#define RIMEADDR_SIZE_RDC 1
#endif

/* This structure holds information about our 1-st hop neighbours. */
struct neighbours {
  /* The ->next pointer is needed since we are placing these on a
Contiki list. */
  struct neighbours  *next;

  /* The ->addr field holds the LL address of the neighbour. */
  uint8_t node_id;
  
  uint8_t rxcounter;
  
 // rimeaddr_t rimeaddr;
  
  //the time out timer for removing old entries
 //struct ctimer ctimer;
};
  
/* This #define defines the maximum amount of neighbors we can remember. */
#define NEIGH_TIMEOUT 10*PERIOD*CLOCK_SECOND

/* This MEMB() definition defines a memory pool from which we allocate
neighbor entries. */
MEMB(neighbours_memb, struct neighbours, MAX_NEIGHS);

/* The neighbors_list is a Contiki list that holds the neighbors we
have seen thus far. */
LIST(neighbours_list);

static void update_neigh(void);
static void remove_neigh(void *n);

#endif
/***********************************************************************/  


/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
  int ret;
  /**************FORTH Modification**********************/
   static uint8_t txcnt = 0;
  /***************************************************/
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
#if NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
#endif /* NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW */

  if(NETSTACK_FRAMER.create() < 0) {
    /* Failed to allocate space for headers */
    PRINTF("nullrdc: send failed, too large header\n");
    ret = MAC_TX_ERR_FATAL;
  } else {

#if NULLRDC_802154_AUTOACK
    int is_broadcast;
    uint8_t dsn;
    dsn = ((uint8_t *)packetbuf_hdrptr())[2] & 0xff;

    NETSTACK_RADIO.prepare(packetbuf_hdrptr(), packetbuf_totlen());

    is_broadcast = rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                                &rimeaddr_null);

    if(NETSTACK_RADIO.receiving_packet() ||
       (!is_broadcast && NETSTACK_RADIO.pending_packet())) {

      /* Currently receiving a packet over air or the radio has
         already received a packet that needs to be read before
         sending with auto ack. */
      ret = MAC_TX_COLLISION;

    } else {
	
      /*******************FORTH Modifications********************************/
    #ifdef NM 
    
    if (!is_broadcast){
	  txcnt++;
	  set_tx_counter(txcnt);
	}
	#endif
       /**********************************************************************/	
    
      switch(NETSTACK_RADIO.transmit(packetbuf_totlen())) {
      case RADIO_TX_OK:
        if(is_broadcast) {
          ret = MAC_TX_OK;
        } else {
          rtimer_clock_t wt;

          /* Check for ack */
          wt = RTIMER_NOW();
          watchdog_periodic();
          while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + ACK_WAIT_TIME));

          ret = MAC_TX_NOACK;
          if(NETSTACK_RADIO.receiving_packet() ||
             NETSTACK_RADIO.pending_packet() ||
             NETSTACK_RADIO.channel_clear() == 0) {
            int len;
            uint8_t ackbuf[ACK_LEN];

            wt = RTIMER_NOW();
            watchdog_periodic();
            while(RTIMER_CLOCK_LT(RTIMER_NOW(),
                                  wt + AFTER_ACK_DETECTED_WAIT_TIME));

            if(NETSTACK_RADIO.pending_packet()) {
              len = NETSTACK_RADIO.read(ackbuf, ACK_LEN);
              if(len == ACK_LEN && ackbuf[2] == dsn) {
                /* Ack received */
                //printf("Ack received\n");
                ret = MAC_TX_OK;
              } else {
                /* Not an ack or ack not for us: collision */
               // printf("/* Not an ack or ack not for us: collision */\n");
                ret = MAC_TX_COLLISION;
              }
            }
          }
        }
        break;
      case RADIO_TX_COLLISION:
        ret = MAC_TX_COLLISION;
        break;
      default:
        ret = MAC_TX_ERR;
        break;
      }
    }

#else /* ! NULLRDC_802154_AUTOACK */
/*******************FORTH Modifications********************************/
    #ifdef NM 
    
    if (!linkaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                                &linkaddr_null)){
	  
	 txcnt++;
	 set_tx_counter(txcnt);
	}
	#endif
/**********************************************************************/	
    switch(NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen())) {
    case RADIO_TX_OK:
      ret = MAC_TX_OK;
      break;
    case RADIO_TX_COLLISION:
      ret = MAC_TX_COLLISION;
      break;
    case RADIO_TX_NOACK:
      ret = MAC_TX_NOACK;
      break;
    default:
      ret = MAC_TX_ERR;
      break;
    }

#endif /* ! NULLRDC_802154_AUTOACK */
  }
  mac_call_sent_callback(sent, ptr, ret, 1);
}
/*---------------------------------------------------------------------------*/
static void
send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  if(buf_list != NULL) {
    queuebuf_to_packetbuf(buf_list->buf);
    send_packet(sent, ptr);
  }
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
/********************************************************/
#ifdef NM
	linkaddr_t tmp;
#endif
/*******************************************************/    
#if NULLRDC_802154_AUTOACK
  if(packetbuf_datalen() == ACK_LEN) {
    /* Ignore ack packets */
    /* PRINTF("nullrdc: ignored ack\n"); */
  } else
#endif /* NULLRDC_802154_AUTOACK */
  if(NETSTACK_FRAMER.parse() < 0) {
    PRINTF("nullrdc: failed to parse %u\n", packetbuf_datalen());
#if NULLRDC_ADDRESS_FILTER
  
  } else if(!linkaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                                         &linkaddr_node_addr) &&
            !linkaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                          &linkaddr_null)) {
    PRINTF("nullrdc: not for us\n");
#endif /* NULLRDC_ADDRESS_FILTER */
  } else {
	  
	 
#if NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW
    /* Check for duplicate packet by comparing the sequence number
       of the incoming packet with the last few ones we saw. */
    int i;
    //printf("aaaa\n");
    for(i = 0; i < MAX_SEQNOS; ++i) {
      if(packetbuf_attr(PACKETBUF_ATTR_PACKET_ID) == received_seqnos[i].seqno &&
         rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_SENDER),
                      &received_seqnos[i].sender)) {
        /* Drop the packet. */
        PRINTF("nullrdc: drop duplicate link layer packet %u\n",
               packetbuf_attr(PACKETBUF_ATTR_PACKET_ID));
        return;
      }
    }
    for(i = MAX_SEQNOS - 1; i > 0; --i) {
      memcpy(&received_seqnos[i], &received_seqnos[i - 1],
             sizeof(struct seqno));
    }
    received_seqnos[0].seqno = packetbuf_attr(PACKETBUF_ATTR_PACKET_ID);
    rimeaddr_copy(&received_seqnos[0].sender,
                  packetbuf_addr(PACKETBUF_ADDR_SENDER));
                  
#endif /* NULLRDC_802154_AUTOACK */
/***************FORTH Modifications*****************/
 #ifdef NM 
	
	linkaddr_copy(&tmp, packetbuf_addr(PACKETBUF_ADDR_SENDER));
   if (!linkaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                          &linkaddr_null)){
	  // printf("**%d**\n", tmp.u8[RIMEADDR_SIZE_RDC-1]); 	  
		update_neigh();
		set_rx_counter();
	   
   }
   #endif //NM
/***************FORTH Modifications*****************/   
    //NETSTACK_MAC.input();
  }
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  return NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static int
off(int keep_radio_on)
{
  if(keep_radio_on) {
    return NETSTACK_RADIO.on();
  } else {
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  on();
 // txcnt = 0;
}
/***********FORTH Modifications****************************/  

#ifdef NM

void set_tx_counter(uint8_t tx_counter)
{
	mac_txrx_counter.txcnt = tx_counter;
	//printf("**%d**\n", mac_txrx_counter.txcnt);
}


static struct mac_counters get_txrx_counters(void)
{
	return mac_txrx_counter;
}	
	
void set_rx_counter(){	
   struct neighbours *n;
   //struct mac_counters cnt2return;
//   rimeaddr_t tmp;
   
  
  
   //mac_txrx_counters.txcnt;
   uint8_t i;
  
   uint16_t listaddr;
   uint8_t rxcnt[MAX_NEIGHS];
   
   for (i=0;i<MAX_NEIGHS;i++){
	   rxcnt[i]=0;
	   
    }	   
    listaddr = 0;
   //  printf("%u**\n", listaddr); 
	//cnt2return.txcnt = txcnt;	   
    i=0;
   for(n = list_head(neighbours_list); n != NULL; n = list_item_next(n)) {
	//memcpy(&tmp, &n->addr, sizeof(uip_lladdr_t));
//each bit set to high now corresponds to the address of a neighbour node from which have've had received packets.
	listaddr |= (1 << (n->node_id-1)) & 0xffff;
	// printf("%u**%u**\n", (1 << (n->node_id-1)),listaddr); 
	rxcnt[i++] = n->rxcounter;
	
   }
  //   printf("%u**\n", listaddr); 
   
	//return cnt2return;
	mac_txrx_counter.listaddr = listaddr;
	for (i=0;i<MAX_NEIGHS;i++){
		mac_txrx_counter.rxcnt[i] = rxcnt[i];
	}		
}


static void update_neigh(void)
 {
	struct neighbours *n;
	linkaddr_t tmp;
	  
	linkaddr_copy(&tmp, packetbuf_addr(PACKETBUF_ADDR_SENDER));
	//  printf("**%d**\n", tmp.u8[RIMEADDR_SIZE_RDC-1]); 

	/* Check if we already know this neighbour. */
	for(n = list_head(neighbours_list); n != NULL; n = list_item_next(n)) {

		
		if(n->node_id == tmp.u8[RIMEADDR_SIZE_RDC-1]) {
			// printf("**%d**\n", tmp.u8[RIMEADDR_SIZE_RDC-1]); 

			 /* Our neighbour was found, so we update the timeout. */
			n->rxcounter++; 
		//	ctimer_set(&n->ctimer, NEIGH_TIMEOUT, remove_neigh, n);
			break;
		}
	}

  /* If n is NULL, this neigh was not found in our list, and we
allocate a new struct neigh from the children_memb memory
pool. */
  if(n == NULL) {
    n = memb_alloc(&neighbours_memb);

    /* If we could not allocate a new neighbour entry, we give up. We
could have reused an old neighbor entry, but we do not do this
for now. */
    if(n != NULL) {
    /* Initialize the fields. */
   // rimeaddr_copy(&tmp, packetbuf_addr(PACKETBUF_ADDR_SENDER));
    n->node_id = tmp.u8[RIMEADDR_SIZE_RDC-1];
    n->rxcounter = 1;
    /* Place the child on the neighbour list at the end of the list. */
    list_add(neighbours_list, n);
    
    
  //  ctimer_set(&n->ctimer, NEIGH_TIMEOUT, remove_neigh, n);
	
    
   
		
		
	//}
	//printf("\n");
    //printf("neighbour %d added on my list!\n", n->node_id);
    }
   }
 /*  printf("My neighbours are: ");
    for(n = list_head(neighbours_list); n != NULL; n = list_item_next(n)) {
	 printf("%d  ", n->node_id);}
	 printf("\n");
   */
}

/*
 * This function is called by the ctimer present in each neighbor
 * table entry. The function removes the neighbor from the table
 * because it has become too old.
 */
static void remove_neigh(void *n)
{
  struct neighbours *e = n;
 //removing old items...
 //printf("now removing node: %d\n",e->addr.addr[5]);
  list_remove(neighbours_list, e);
  memb_free(&neighbours_list, e);
  
}
#endif
/*****************************************************/  

/*---------------------------------------------------------------------------*/
const struct rdc_driver nullrdc_dl_driver = {
  "nullrdc_dl",
  init,
  send_packet,
  send_list,
  packet_input,
  on,
  off,
  channel_check_interval,
  /***********FORTH Modification****************************/  
  #ifdef NM
	get_txrx_counters,
  #endif
  /***************************************************/  

};
/*---------------------------------------------------------------------------*/