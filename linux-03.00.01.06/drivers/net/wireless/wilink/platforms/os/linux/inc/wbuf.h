/*
 * wbuf.h
 *
 * Copyright(c) 1998 - 2010 Texas Instruments. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Texas Instruments nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***************************************************************************/
/*									   */
/*    MODULE:	wbuf.h						       */
/*    PURPOSE:	manages the allocation/free and field access of the WBUF   */
/*									   */
/***************************************************************************/
#ifndef _WBUF_H_
#define _WBUF_H_

#include <linux/version.h>
#include <linux/skbuff.h>
#include "tidef.h"
#include "queue.h"

typedef void WBUF;

/* Packet types */
typedef enum {
	TX_PKT_TYPE_MGMT = 0x01,
	TX_PKT_TYPE_EAPOL = 0x02,
	/* Data packets from the Network interface. */
	TX_PKT_TYPE_ETHER = 0x04,
	/* Currently includes Null and IAPP packets. */
	TX_PKT_TYPE_WLAN_DATA = 0x08
} TX_PKT_TYPE;

/* Tx-Packet-Flag */
#define TX_PKT_FLAGS_LINK_TEST		0x1
/* Add padding before data buffer for WSPI overhead */
#define WSPI_PAD_BYTES              	16

/* user callback definition (in tx complete) */
typedef void *CB_ARG;
typedef void (*CB_FUNC) (CB_ARG cb_arg);

/*
 * wbuf user fields: 
 */
typedef struct {
	/* The header used for queueing the WBUF            */
	TQueNodeHdr queNodeHdr;
	/* wbuf packet type                                */
	TX_PKT_TYPE pktType;
	/* callback function to use in tx complete         */
	CB_FUNC cb_func;
	/* callback argument to use in tx complete         */
	CB_ARG cb_arg;
	/* WLAN TID (traffic identity)                     */
	TI_UINT8 Tid;
	/* WLAN header length, not including alignment pad. */
	TI_UINT8 hdrLen;
	/* Some application flags, see Tx-Packet-Flags defs above. */
	TI_UINT8 flags;
} WBUF_PARAMS;

#define WBUF_DATA(pWbuf)	( ((struct sk_buff *)(pWbuf))->data )
#define WBUF_LEN(pWbuf)		( ((struct sk_buff *)(pWbuf))->len )
#define WBUF_PRIORITY(pWbuf)	( ((struct sk_buff *)(pWbuf))->priority )
#define WBUF_DEV(pWbuf)		( ((struct sk_buff *)(pWbuf))->dev )
#define WBUF_DEV_SET(pWbuf,pDev)    	\
	( ((struct sk_buff *)(pWbuf))->dev) = ((struct net_device *)(pDev))
#ifdef  HOST_PLATFORM_OMAP3430 || HOST_PLATFORM_OMAP3530 \
	|| HOST_PLATFORM_ZOOM2 || HOST_PLATFORM_ZOOM1
#define WBUF_STAMP(pWbuf)	( ((struct sk_buff *)(pWbuf))->tstamp.tv.nsec )
#else
#define WBUF_STAMP(pWbuf)	( ((struct sk_buff *)(pWbuf))->tstamp.off_usec )
#endif
#define WBUF_CB(pWbuf)		( ((struct sk_buff *)(pWbuf))->cb )
#define WBUF_PKT_TYPE(pWbuf)	( ((WBUF_PARAMS *)&(WBUF_CB(pWbuf)))->pktType )
#define WBUF_CB_FUNC(pWbuf)     ( ((WBUF_PARAMS *)&(WBUF_CB(pWbuf)))->cb_func )
#define WBUF_CB_ARG(pWbuf)      ( ((WBUF_PARAMS *)&(WBUF_CB(pWbuf)))->cb_arg )
#define WBUF_TID(pWbuf)	        ( ((WBUF_PARAMS *)&(WBUF_CB(pWbuf)))->Tid )
#define WBUF_HDR_LEN(pWbuf)	( ((WBUF_PARAMS *)&(WBUF_CB(pWbuf)))->hdrLen )
#define WBUF_FLAGS(pWbuf)	( ((WBUF_PARAMS *)&(WBUF_CB(pWbuf)))->flags )

/* The offset of the node-header field from the WBUF entry (for queueing) */
#define WBUF_NODE_HDR_OFFSET    \
		( (unsigned long) &( ( (WBUF_PARAMS *) \
		&( ( (struct sk_buff *)0 )->cb ) )->queNodeHdr ) )

/* 
 * Allocate WBUF for Tx/Rx packets.
 * Add 16 bytes before the data buffer for WSPI overhead!
 */
static inline WBUF *WbufAlloc(TI_HANDLE hOs, TI_UINT32 len)
{
	WBUF *pWbuf = alloc_skb(len + WSPI_PAD_BYTES, GFP_ATOMIC);
	WBUF_DATA(pWbuf) += WSPI_PAD_BYTES;
	return pWbuf;
}

#define WbufFree(hOs, pWbuf)		\
		( dev_kfree_skb((struct sk_buff *)pWbuf) )
#define WbufReserve(hOs, pWbuf,len)	\
		( skb_reserve((struct sk_buff *)pWbuf,(int)len) )

#endif