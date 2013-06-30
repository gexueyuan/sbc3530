/***************************************************************************
**+----------------------------------------------------------------------+**
**|                                ****                                  |**
**|                                ****                                  |**
**|                                ******o***                            |**
**|                          ********_///_****                           |**
**|                           ***** /_//_/ ****                          |**
**|                            ** ** (__/ ****                           |**
**|                                *********                             |**
**|                                 ****                                 |**
**|                                  ***                                 |**
**|                                                                      |**
**|     Copyright (c) 1998 - 2009 Texas Instruments Incorporated         |**
**|                        ALL RIGHTS RESERVED                           |**
**|                                                                      |**
**| Permission is hereby granted to licensees of Texas Instruments       |**
**| Incorporated (TI) products to use this computer program for the sole |**
**| purpose of implementing a licensee product based on TI products.     |**
**| No other rights to reproduce, use, or disseminate this computer      |**
**| program, whether in part or in whole, are granted.                   |**
**|                                                                      |**
**| TI makes no representation or warranties with respect to the         |**
**| performance of this computer program, and specifically disclaims     |**
**| any responsibility for any damages, special or consequential,        |**
**| connected with the use of this program.                              |**
**|                                                                      |**
**+----------------------------------------------------------------------+**
***************************************************************************/

/** \file  buf.c
 *  \brief Linux buf implementation.
 *
 *  \see   
 */

#include "tidef.h"
#include "RxBuf_linux.h"
#include <linux/netdevice.h>
/*--------------------------------------------------------------------------------------*/
/* 
 * Allocate BUF Rx packets.
 * Add 16 bytes before the data buffer for WSPI overhead!
 */
void *RxBufAlloc(TI_HANDLE hOs, TI_UINT32 len, PacketClassTag_e ePacketClassTag)
{
	rx_head_t *rx_head;
	TI_UINT32 alloc_len =
	    len + WSPI_PAD_BYTES + PAYLOAD_ALIGN_PAD_BYTES +
	    RX_HEAD_LEN_ALIGNED;
	struct sk_buff *skb = alloc_skb(alloc_len, GFP_ATOMIC);

	if (skb == NULL) {
		printk("RxBufAlloc(): alloc_skb failed\n");
		return NULL;
	}
	rx_head = (rx_head_t *) skb->head;

	rx_head->skb = skb;
	skb_reserve(skb, RX_HEAD_LEN_ALIGNED + WSPI_PAD_BYTES);
/*
	printk("-->> RxBufAlloc(len=%d)  skb=0x%x skb->data=0x%x skb->head=0x%x skb->len=%d\n",
		   (int)len, (int)skb, (int)skb->data, (int)skb->head, (int)skb->len);
*/
	return skb->data;

}

/*--------------------------------------------------------------------------------------*/

inline void RxBufFree(TI_HANDLE hOs, void *pBuf)
{
	unsigned char *pdata =
	    (unsigned char *)((TI_UINT32) pBuf & ~(TI_UINT32) 0x3);
	rx_head_t *rx_head =
	    (rx_head_t *) (pdata - WSPI_PAD_BYTES - RX_HEAD_LEN_ALIGNED);
	struct sk_buff *skb = rx_head->skb;

#ifdef TI_DBG
	if ((TI_UINT32) pBuf & 0x3) {
		if ((TI_UINT32) pBuf - (TI_UINT32) skb->data != 2) {
			printk
			    ("RxBufFree() address error skb=0x%x skb->data=0x%x pPacket=0x%x !!!\n",
			     (int)skb, (int)skb->data, (int)pBuf);
		}
	} else {
		if ((TI_UINT32) skb->data != (TI_UINT32) pBuf) {
			printk
			    ("RxBufFree() address error skb=0x%x skb->data=0x%x pPacket=0x%x !!!\n",
			     (int)skb, (int)skb->data, (int)pBuf);
		}
	}
#endif
/*
	printk("-->> RxBufFree()  skb=0x%x skb->data=0x%x skb->head=0x%x skb->len=%d\n",
		   (int)skb, (int)skb->data, (int)skb->head, (int)skb->len);
*/
	dev_kfree_skb(skb);
}
