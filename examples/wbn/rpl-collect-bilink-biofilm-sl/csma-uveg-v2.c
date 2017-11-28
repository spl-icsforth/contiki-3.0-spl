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
 * $Id: csma.c,v 1.27 2011/01/25 14:24:38 adamdunkels Exp $
 */

/**
 * \file
 *         A Carrier Sense Multiple Access (CSMA) MAC layer
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "net/mac/csma.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "dev/leds.h"
#include "sys/ctimer.h"
#include "sys/clock.h"

#include "lib/random.h"

#include "net/netstack.h"

#include "lib/list.h"
#include "lib/memb.h"

#include <string.h>

#include <stdio.h>

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else /* DEBUG */
#define PRINTF(...)
#endif /* DEBUG */

#ifndef CSMA_MAX_MAC_TRANSMISSIONS
#ifdef CSMA_CONF_MAX_MAC_TRANSMISSIONS
#define CSMA_MAX_MAC_TRANSMISSIONS CSMA_CONF_MAX_MAC_TRANSMISSIONS
#else
#define CSMA_MAX_MAC_TRANSMISSIONS 3
#endif /* CSMA_CONF_MAX_MAC_TRANSMISSIONS */
#endif /* CSMA_MAX_MAC_TRANSMISSIONS */

#if CSMA_MAX_MAC_TRANSMISSIONS < 1
#error CSMA_CONF_MAX_MAC_TRANSMISSIONS must be at least 1.
#error Change CSMA_CONF_MAX_MAC_TRANSMISSIONS in contiki-conf.h or in your Makefile.
#endif /* CSMA_CONF_MAX_MAC_TRANSMISSIONS < 1 */

/* Packet metadata */
struct qbuf_metadata {
  mac_callback_t sent;
  void *cptr;
  uint8_t max_transmissions;
};

/* Every neighbor has its own packet queue */
struct neighbor_queue {
  struct neighbor_queue *next;
  rimeaddr_t addr;
  struct ctimer transmit_timer;
  uint8_t transmissions;
  uint8_t collisions, deferrals;
  LIST_STRUCT(queued_packet_list);
};


/**********  UVEG modifications ******************/
/***   Necessary code for packet modifications at
 ***    this level.                            ***/
/*
struct sensor_datamsg{
  int measured_rssi;
  uint16_t packet_num;
  rimeaddr_t destination;
  rimeaddr_t source;
}sensor_datamsg;



struct queuebuf {
#if QUEUEBUF_DEBUG
  struct queuebuf *next;
  const char *file;
  int line;
  clock_time_t time;
#endif // QUEUEBUF_DEBUG 

#if WITH_SWAP
  enum {IN_RAM, IN_CFS} location;
  union {
#endif
    struct queuebuf_data *ram_ptr;
#if WITH_SWAP
    int swap_id;
  };
#endif
};

/* The actual queuebuf data //

struct queuebuf_data {
  uint16_t len;
  uint8_t data[PACKETBUF_SIZE];
  struct packetbuf_attr attrs[PACKETBUF_NUM_ATTRS];
  struct packetbuf_addr addrs[PACKETBUF_NUM_ADDRS];
};
*/
/*****************************************************/

/* The maximum number of co-existing neighbor queues */
#ifdef CSMA_CONF_MAX_NEIGHBOR_QUEUES
#define CSMA_MAX_NEIGHBOR_QUEUES CSMA_CONF_MAX_NEIGHBOR_QUEUES
#else
#define CSMA_MAX_NEIGHBOR_QUEUES 2
#endif /* CSMA_CONF_MAX_NEIGHBOR_QUEUES */

#define MAX_QUEUED_PACKETS QUEUEBUF_NUM
MEMB(neighbor_memb, struct neighbor_queue, CSMA_MAX_NEIGHBOR_QUEUES);
MEMB(packet_memb, struct rdc_buf_list, MAX_QUEUED_PACKETS);
MEMB(metadata_memb, struct qbuf_metadata, MAX_QUEUED_PACKETS);
LIST(neighbor_list);


/**********  UVEG MAC modifications ******************/
#ifndef NUM_NEIGHBORS_CONF_UVEGCSMA
/***********FORTH Modification****************************/  
//#define NUM_NEIGHBORS_UVEGCSMA        9
 
#define NUM_NEIGHBORS_UVEGCSMA        NET_SIZE
/************************************************************/  
#else
#define NUM_NEIGHBORS_UVEGCSMA        NUM_NEIGHBORS_CONF_UVEGCSMA
#endif

static uint16_t  threshold_vectors[NUM_NEIGHBORS_UVEGCSMA];
static uint16_t contention_window = 0.5;

/***********FORTH Modification****************************/  
#ifdef RIMEADDR_CONF_SIZE //ipv6 case
#define RIMEADDR_SIZE_MAC RIMEADDR_CONF_SIZE
#else
#define RIMEADDR_SIZE_MAC 1
#endif
/*************************************************/




static void packet_sent(void *ptr, int status, int num_transmissions);
static void transmit_packet_list(void *ptr);
//static int transmit_or_not(const rimeaddr_t *addr);

/*---------------------------------------------------------------------------*/
static struct
neighbor_queue *neighbor_queue_from_addr(const rimeaddr_t *addr) {
  struct neighbor_queue *n = list_head(neighbor_list);
  while(n != NULL) {
    if(rimeaddr_cmp(&n->addr, addr)) {
      return n;
    }
    n = list_item_next(n);
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
static clock_time_t
default_timebase(void)
{
  clock_time_t time;
  /* The retransmission time must be proportional to the channel
     check interval of the underlying radio duty cycling layer. */
  time = NETSTACK_RDC.channel_check_interval();

  /* If the radio duty cycle has no channel check interval (i.e., it
     does not turn the radio off), we make the retransmission time
     proportional to the configured MAC channel check rate. */
  if(time == 0) {
    time = CLOCK_SECOND / NETSTACK_RDC_CHANNEL_CHECK_RATE;
  }
  return time;
}
/*---------------------------------------------------------------------------*/

/***************************    UVEG MODIFICATIONS    ************************/
static void
transmit_packet_list(void *ptr)
{
  struct neighbor_queue *n = ptr;
  int send;//, rssi_aux;//, aux;
 // uint16_t pack_num;
 // clock_time_t time = 0;
    
  if(n) {
    
   // rssi_aux=NETSTACK_RADIO.rssi()-45;
    
  //  printf("Me: %d.\n",rimeaddr_node_addr.u8[7]);	
  //  printf("Neighbor: %d.\n",n->addr.u8[RIMEADDR_SIZE_MAC-1]);
  //printf("Threshold: %d.\n",threshold_vectors[rimeaddr_node_addr.u8[7]-1][n->addr.u8[RIMEADDR_SIZE_MAC-1]-1]);
  /***************************FORTH Modification****************************/  
    //if((n->addr.u8[RIMEADDR_SIZE_MAC-1]>0) && (n->addr.u8[RIMEADDR_SIZE_MAC-1]<=NUM_NEIGHBORS_UVEGCSMA) && (rssi_aux<threshold_vectors[n->addr.u8[RIMEADDR_SIZE_MAC-1]-1]))
    if((n->addr.u8[RIMEADDR_SIZE_MAC-1]<=NUM_NEIGHBORS_UVEGCSMA) && ((NETSTACK_RADIO.rssi()-45)<threshold_vectors[n->addr.u8[RIMEADDR_SIZE_MAC-1]-1]))
    /***********************************************************************/  
    {
      send=1;
    }
    else
    {
      send=0;
    }
        
    if (send==1)
    {
      struct rdc_buf_list *q = list_head(n->queued_packet_list);
                      
      if(q != NULL) {
        
        /***********   UVEG MODIFICATIONS   *************/
        /*** Use this code to modify the data packet  ***/
        /*** at this level                            ***/
        /*
        struct queuebuf *qb = (struct queuebuf *) q->buf;
        if(qb != NULL) {
          struct queuebuf_data *qbd = qb->ram_ptr;
          if(qbd != NULL) {
            printf("RSSI FUNCION: %d\n",rssi_aux);
            qbd->data[7] = rssi_aux >> 8; //msb
            qbd->data[6] = rssi_aux & 0xff; //lsb
            pack_num = qbd->data[9] <<8 | qbd->data[8];
            printf("PACK_NUM: %d\n",pack_num);
            aux = qbd->data[7] <<8 | qbd->data[6];
            printf("RSSI: %d\n",aux);
            pack_num = qbd->data[5] <<8 | qbd->data[4];
            printf("SOURCE: %d\n",pack_num);
            pack_num = qbd->data[3] <<8 | qbd->data[2];
            printf("DESTINATION: %d\n",pack_num);
          }
          
        }
        */
        /***********************************************/
        
     //   PRINTF("csma: preparing number %d %p, queue len %d\n", n->transmissions, q,
//        list_length(n->queued_packet_list));
        
        /* Send packets in the neighbor's list */
        //leds_toggle(LEDS_GREEN);
        ctimer_set(&n->transmit_timer, random_rand()%(CLOCK_SECOND/20), transmit_packet_list, n);
        NETSTACK_RDC.send_list(packet_sent, n, q);
      }
    }
    else
    {
      // We wait for a time, randomly chosen between 0 and "contention_window" milliseconds. 
     n->deferrals++;
    //  time=random_rand()%(CLOCK_SECOND/20);
      
  //    PRINTF("Retransmission time: %u.\n", time);
      ctimer_set(&n->transmit_timer, random_rand()%(CLOCK_SECOND/20), transmit_packet_list, n);
      
 
    }
  }
}
/*---------------------------------------------------------------------------*/


static void
free_first_packet(struct neighbor_queue *n)
{
  struct rdc_buf_list *q = list_head(n->queued_packet_list);
  if(q != NULL) {
    /* Remove first packet from list and deallocate */
    queuebuf_free(q->buf);
    list_pop(n->queued_packet_list);
    memb_free(&metadata_memb, q->ptr);
    memb_free(&packet_memb, q);
 //   PRINTF("csma: free_queued_packet, queue length %d\n",
  //      list_length(n->queued_packet_list));
    if(list_head(n->queued_packet_list)) {
      /* There is a next packet. We reset current tx information */
      n->transmissions = 0;
      n->collisions = 0;
      n->deferrals = 0;
      /* Set a timer for next transmissions */
      ctimer_set(&n->transmit_timer, default_timebase(), transmit_packet_list, n);
      //ctimer_set(&n->transmit_timer, random_rand()%(CLOCK_SECOND/20), transmit_packet_list, n);
      
    } else {
      /* This was the last packet in the queue, we free the neighbor */
      ctimer_stop(&n->transmit_timer);
      list_remove(neighbor_list, n);
      memb_free(&neighbor_memb, n);
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
packet_sent(void *ptr, int status, int num_transmissions)
{
  struct neighbor_queue *n = ptr;
  struct rdc_buf_list *q = list_head(n->queued_packet_list);
  struct qbuf_metadata *metadata = (struct qbuf_metadata *)q->ptr;
  clock_time_t time = 0;
  mac_callback_t sent;
  void *cptr;
  int num_tx;
  int backoff_transmissions;

  
  
  switch(status) {
  case MAC_TX_COLLISION:
 //   PRINTF("PACKET_SENT CSMA COLLISION.\n");
 //  printf("PACKET_SENT CSMA COLLISION.\n");
      n->collisions++;
      break;
  case MAC_TX_OK:
  case MAC_TX_NOACK:
 //   PRINTF("PACKET_SENT CSMA OK_NOACK STATUS:%d.\n",status);
   // printf("PACKET_SENT CSMA OK_NOACK STATUS:%d.\n",status);
    n->transmissions++;
    break;
  case MAC_TX_DEFERRED:
  //  PRINTF("PACKET_SENT CSMA DEFERRED.\n");
  //  printf("PACKET_SENT CSMA DEFERRED.\n");
    n->deferrals++;
    break;
  }

  sent = metadata->sent;
  cptr = metadata->cptr;
  num_tx = n->transmissions;
  if(status == MAC_TX_COLLISION ||
     status == MAC_TX_NOACK) {
    /* If the transmission was not performed because of a collision or
       noack, we must retransmit the packet. */
    
  /*  switch(status) {
    case MAC_TX_COLLISION:
      PRINTF("csma: rexmit collision %d\n", n->transmissions);
      break;
    case MAC_TX_NOACK:
      PRINTF("csma: rexmit noack %d\n", n->transmissions);
      break;
    default:
      PRINTF("csma: rexmit err %d, %d\n", status, n->transmissions);
    }*/

    /* The retransmission time must be proportional to the channel
       check interval of the underlying radio duty cycling layer. */
    time = default_timebase();

    /* The retransmission time uses a linear backoff so that the
       interval between the transmissions increase with each
       retransmit. */
    backoff_transmissions = n->transmissions + 1;

    /* Clamp the number of backoffs so that we don't get a too long
       timeout here, since that will delay all packets in the
       queue. */
    if(backoff_transmissions > 3) {
      backoff_transmissions = 3;
    }

    time = time + (random_rand() % (backoff_transmissions * time));
   // PRINTF("CSMA: Max retransmissions: %d\n", metadata->max_transmissions);
    if(n->transmissions < metadata->max_transmissions) {
   //   PRINTF("csma: retransmitting with time %lu %p\n", time, q);
      ctimer_set(&n->transmit_timer, time,
                 transmit_packet_list, n);
      /* This is needed to correctly attribute energy that we spent
         transmitting this packet. */
      queuebuf_update_attr_from_packetbuf(q->buf);
    } else {
  //    PRINTF("csma: drop with status %d after %d transmissions, %d collisions\n",
  //           status, n->transmissions, n->collisions);
      free_first_packet(n);
    //  PRINTF("CALLBACK DESDE PACKET_SENT. STATUS: %d\n",status);
      mac_call_sent_callback(sent, cptr, status, num_tx);
    }
  } else {
    if(status == MAC_TX_OK) {
      PRINTF("csma: rexmit ok %d\n", n->transmissions);
    } else {
      PRINTF("csma: rexmit failed %d: %d\n", n->transmissions, status);
    }
    free_first_packet(n);
  //  PRINTF("CALLBACK DESDE PACKET_SENT. STATUS: %d\n",status);
    mac_call_sent_callback(sent, cptr, status, num_tx);
  }
}
/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
  struct rdc_buf_list *q;
  struct neighbor_queue *n;
  static uint16_t seqno;
  //printf("CSMA.c file.\n");
  
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, seqno++);
  
  /* If the packet is a broadcast, do not allocate a queue
     entry. Instead, just send it out.  */
  if(!rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                   &rimeaddr_null)) {
    const rimeaddr_t *addr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);

    /* Look for the neighbor entry */
    n = neighbor_queue_from_addr(addr);
    if(n == NULL) {
      /* Allocate a new neighbor entry */
      n = memb_alloc(&neighbor_memb);
      if(n != NULL) {
        /* Init neighbor entry */
        rimeaddr_copy(&n->addr, addr);
        n->transmissions = 0;
        n->collisions = 0;
        n->deferrals = 0;
        /* Init packet list for this neighbor */
        LIST_STRUCT_INIT(n, queued_packet_list);
        /* Add neighbor to the list */
        list_add(neighbor_list, n);
      }
    }

    if(n != NULL) {
      /* Add packet to the neighbor's queue */
      q = memb_alloc(&packet_memb);
      if(q != NULL) {
        q->ptr = memb_alloc(&metadata_memb);
        if(q->ptr != NULL) {
          q->buf = queuebuf_new_from_packetbuf();
          if(q->buf != NULL) {
            struct qbuf_metadata *metadata = (struct qbuf_metadata *)q->ptr;
            /* Neighbor and packet successfully allocated */
            if(packetbuf_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS) == 0) {
              /* Use default configuration for max transmissions */
              metadata->max_transmissions = CSMA_MAX_MAC_TRANSMISSIONS;
            } else {
              metadata->max_transmissions =
                  packetbuf_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS);
            }
            metadata->sent = sent;
            metadata->cptr = ptr;

            if(packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) ==
                PACKETBUF_ATTR_PACKET_TYPE_ACK) {
              list_push(n->queued_packet_list, q);
            } else {
  //            PRINTF("CSMA: list_add\n");
              list_add(n->queued_packet_list, q);
            }

            /* If q is the first packet in the neighbor's queue, send asap */
            if(list_head(n->queued_packet_list) == q) {
              ctimer_set(&n->transmit_timer, 0, transmit_packet_list, n);
            }
            return;
          }
          memb_free(&metadata_memb, q->ptr);
  //        PRINTF("csma: could not allocate queuebuf, dropping packet\n");
        }
        memb_free(&packet_memb, q);
        PRINTF("csma: could not allocate queuebuf, dropping packet\n");
      }
      /* The packet allocation failed. Remove and free neighbor entry if empty. */
      if(list_length(n->queued_packet_list) == 0) {
        list_remove(neighbor_list, n);
        memb_free(&neighbor_memb, n);
      }
   //   PRINTF("csma: could not allocate packet, dropping packet\n");
    }// else {
     // PRINTF("csma: could not allocate neighbor, dropping packet\n");
   // }
    mac_call_sent_callback(sent, ptr, MAC_TX_ERR, 1);
  } else {
    //printf("csma: send broadcast\n");
    NETSTACK_RDC.send(sent, ptr);
  }
}
/*---------------------------------------------------------------------------*/
static void
input_packet(void)
{
  //printf("csma: input packet\n");
  NETSTACK_NETWORK.input();
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  return NETSTACK_RDC.on();
}
/*---------------------------------------------------------------------------*/
static int
off(int keep_radio_on)
{
  return NETSTACK_RDC.off(keep_radio_on);
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  if(NETSTACK_RDC.channel_check_interval) {
    return NETSTACK_RDC.channel_check_interval();
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  
  /**********  UVEG MODIFICATIONS   ********/
  int i;//,j;
  
 // PRINTF("Num neighbors: %d.\n",NUM_NEIGHBORS_UVEGCSMA);
  for(i=0;i<NUM_NEIGHBORS_UVEGCSMA;i++)
  {
   // for(j=0;j<NUM_NEIGHBORS_UVEGCSMA;j++)
   // {
       threshold_vectors[i]=-77;
   // }
  }
  if (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 1){
	  threshold_vectors[10] = -79;  // 3.2m
		threshold_vectors[11] = -89;
		threshold_vectors[12] = -84;
	  }
	  
else if	 (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 2) {
	threshold_vectors[9] = -84;
	threshold_vectors[8] = -84;
	threshold_vectors[2] = -66;
	threshold_vectors[5] = -85;  // 5.5m
	threshold_vectors[3] = -85;
	threshold_vectors[4] = -85;
	
	}
else if	 (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 3) {
	threshold_vectors[9] = -84;

	threshold_vectors[8] = -84;
	threshold_vectors[5] = -85;

	threshold_vectors[3] = -85;

	threshold_vectors[4] = -85;
	threshold_vectors[1] = -66;
	
	}
	else if	 (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 4) {
	threshold_vectors[1] = -85;
	threshold_vectors[2] = -85;
	threshold_vectors[11] = -84;
	threshold_vectors[9] = -89;
	threshold_vectors[8] = -89;
	threshold_vectors[7] = -83;  // 4.7m
	threshold_vectors[6] = -83;
	threshold_vectors[5] = -66;
	threshold_vectors[4] = -66;
	
	}
	
	else if	 (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 5) {
	threshold_vectors[1] = -85;
	threshold_vectors[2] = -85;
	threshold_vectors[3] = -66;
	threshold_vectors[11] = -84;
	threshold_vectors[9] = -89;

	threshold_vectors[8] = -89;
	threshold_vectors[7] = -83;
	threshold_vectors[6] = -83;
	threshold_vectors[5] = -66;
	
	}
	else if	 (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 6) {
	threshold_vectors[1] = -85;  // 5.5m
	threshold_vectors[2] = -85;
	threshold_vectors[3] = -66;
	threshold_vectors[4] = -66;
	threshold_vectors[11] = -84;   // 5m
	threshold_vectors[9] = -89;  // 8m
	threshold_vectors[8] = -89;
	threshold_vectors[7] = -83;
	threshold_vectors[6] = -83;
	
	}
	else if	 (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 7) {
	
	threshold_vectors[3] = -83;
	threshold_vectors[4] = -83;
	threshold_vectors[5] = -83;
	threshold_vectors[7] = -66;  // 1m
	threshold_vectors[11] = -87;
	threshold_vectors[8] = -87;  // 6.5m	
	threshold_vectors[9] = -87;
	}
	else if	 (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 8) {
	threshold_vectors[3] = -83;  // 4.7m
	threshold_vectors[4] = -83;
	threshold_vectors[5] = -83;
	threshold_vectors[11] = -87;
	threshold_vectors[8] = -87;

	threshold_vectors[9] = -87;
	threshold_vectors[6] = -66;  // 1m
	
	}
	else if	 (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 9) {
	threshold_vectors[3] = -89;
	threshold_vectors[4] = -89;
	threshold_vectors[5] = -89;

	threshold_vectors[6] = -87;  // 6.5m
	threshold_vectors[7] = -87;
	threshold_vectors[9] = -66;
	threshold_vectors[1] = -84;
	threshold_vectors[2] = -84;
	
	}
	else if	 (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 10) {
	threshold_vectors[5] = -89;  // 8m
	threshold_vectors[3] = -89;
	threshold_vectors[4] = -89;

	threshold_vectors[6] = -87;
	threshold_vectors[7] = -87;
	threshold_vectors[1] = -84;
	threshold_vectors[2] = -84;

	threshold_vectors[8] = -66;

	
	}
	else if	 (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 11) {
	
	threshold_vectors[12] = -85;
	threshold_vectors[0] = -79;  // 3.2m
	threshold_vectors[14] = -85;
	threshold_vectors[13] = -91;  // 10m
	threshold_vectors[11] = -84;
	}
	else if	 (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 12) {
	
	
	threshold_vectors[6] = -87;
	threshold_vectors[7] = -87;
	threshold_vectors[5] = -84;   // 5m
	threshold_vectors[4] = -84;
	threshold_vectors[3] = -84;

	threshold_vectors[10] = -84;
	threshold_vectors[0] = -89;
	}
	else if	 (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 13) {
	threshold_vectors[10] = -85;
	threshold_vectors[0] = -84;

	threshold_vectors[13] = -84;
	threshold_vectors[14] = -66;
	
	}
	else if	 (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 14) {
		threshold_vectors[10] = -91;  // 10m
		threshold_vectors[12] = -84;
		threshold_vectors[15] = -84;
		threshold_vectors[14] = -84;

	
	}
	else if	 (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 15) {
	threshold_vectors[10] = -85;
	threshold_vectors[13] = -84;
	threshold_vectors[12] = -66;
	threshold_vectors[15] = -91;
	}
	else if  (rimeaddr_node_addr.u8[RIMEADDR_SIZE_MAC-1] == 16){
	threshold_vectors[14] = -91;
	threshold_vectors[13] = -84;

}


  /*****************************************/  
  
  memb_init(&packet_memb);
  memb_init(&metadata_memb);
  memb_init(&neighbor_memb);
}



/**********   UVEG MODIFICATIONS   *******************************************
static int transmit_or_not(const rimeaddr_t *addr)
{
  int rssi_aux;
  
  rssi_aux=NETSTACK_RADIO.rssi();
  printf("RSSI: %d.\n",rssi_aux);
 /****************FORTH modification*************************************
 // printf("Neighbor: %d.\n",addr->u8[0]);
 // printf("Threshold: %d.\n",threshold_vectors[addr->u8[0]]);
  //if((addr->u8[0]>0) && (addr->u8[0]<NUM_NEIGHBORS_UVEGCSMA) && (rssi_aux<threshold_vectors[addr->u8[0]]))
 printf("Neighbor: %d.\n",addr->u8[RIMEADDR_SIZE_MAC-1]);
  printf("Threshold: %d.\n",threshold_vectors[rimeaddr_node_addr.u8[0]-1][addr->u8[RIMEADDR_SIZE_MAC-1]-1]);
  if((addr->u8[RIMEADDR_SIZE_MAC-1]>0) && (addr->u8[RIMEADDR_SIZE_MAC-1]<=NUM_NEIGHBORS_UVEGCSMA) && (rssi_aux<threshold_vectors[rimeaddr_node_addr.u8[0]-1][addr->u8[RIMEADDR_SIZE_MAC-1]-1]))
 /***********************************************************************

  {
    return 1;
  }
  else
  {
    return 0;
  }
}
/***************************************************************************/

/*---------------------------------------------------------------------------*/
/****************FORTH modification*************************************/
//const struct mac_driver csma_uveg_driver = {
//  "CSMA_UVEG",
/***********************************************************************/  
const struct mac_driver csma_uveg_driver = {
  "CSMA_UVEG",
  init,
  send_packet,
  input_packet,
  on,
  off,
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/


