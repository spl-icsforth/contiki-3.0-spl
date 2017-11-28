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
 */

#ifndef __NM_COMMON_H__
#define __NM_COMMON_H__

#include "contiki.h"
#include "net/linkaddr.h"
//#include "net/rime/rimeaddr.h"



//2013-11-27: nancy pan@ forth-ics
//static uint8_t get_meannf();
uint8_t get_nf(void);
PROCESS_NAME(nm_common_process);
#define TRACEROUTE_LEN 2

#ifdef NM
	void nm_input();
	#ifdef TRACE_ROUTE
		
	#ifdef TRACE_LQ	
		void update_uip6pck(int nwk_header, int rssi_lqi);
	
	#else
		void update_uip6pck(int nwk_header);
	#endif //LINK_QUALITY
    #endif
    #ifdef WITH_DL
	//	uint8_t get_nmlist_length();
	
	//	uint8_t get_nmlist(uip_lladdr_t *listaddr, uint8_t size);

    #endif
#endif



#endif /* __NM_COMMON_H__ */
