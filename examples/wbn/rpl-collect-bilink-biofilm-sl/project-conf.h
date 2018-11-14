/*
 * Copyright (c) 2013-2014, FORTH-ICS, UVEG,KTH
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
 * 
 */

/**
 * \file
 *         Project specific configuration defines for the HBioNETs WBN protocol stack.
 *
 *
 * \author
 *         Nancy Panousopoulou - <apanouso@ics.forth.gr>
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_


/********************FORTH Modifications nancy pan********************/
//#define N 1 //number of measurements per sampling iteration.
#define NETSTACK_CONF_RDC      nullrdc_dl_driver
#define NET_SIZE 16 //total!
//We will not use the csma_uveg_driver right now
//#define NETSTACK_CONF_MAC      csma_uveg_driver
#define SINK_ID 16
#define MAX_HTL 5
#define MAX_CHILDREN 16
#define MAX_NEIGHS 10
#define MAX_FREQS 10
#define NNN 1 //change this if you want more measurements / sampling iteration.


/* KTH (John) Ram-saving - reduce the size of the
 * memory that is allocated for queue buffers in
 * Contiki
 */
#ifdef QUEUEBUF_CONF_NUM
#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM                4//8
#endif


//#define 


#ifdef	QUEUEBUF_CONF_NUM
#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM                4//16
#endif /* QUEUEBUF_CONF_NUM */


//#define RPL_CONF_DIO_INTERVAL_MIN 3

//#define RPL_CONF_DIO_INTERVAL_DOUBLINGS 15

/* configure number of neighbors and routes */
#ifdef UIP_CONF_DS6_NBR_NBU
#undef UIP_CONF_DS6_NBR_NBU
#define UIP_CONF_DS6_NBR_NBU     10//20 KTH (John) RAM-saving
#endif /* UIP_CONF_DS6_NBR_NBU */

#ifdef UIP_CONF_DS6_ROUTE_NBU
#undef UIP_CONF_DS6_ROUTE_NBU
#define UIP_CONF_DS6_ROUTE_NBU   10//20 KTH (John) RAM-saving
#endif /* UIP_CONF_DS6_ROUTE_NBU */



#endif
