/*
 * Gadget Driver for Android MTP
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __F_MTP_H
#define __F_MTP_H


#define MTP_MAX_PACKET_LEN_FROM_APP 22

#define	MTP_ACM_ENABLE		0
#define	MTP_ONLY_ENABLE		1
#define	MTP_DISABLE		2
#define	MTP_CLEAR_HALT		3
#define	MTP_WRITE_INT_DATA	4
#define SET_MTP_USER_PID	5
#define GET_SETUP_DATA		6
#define SET_SETUP_DATA		7
#define SEND_RESET_ACK		8
#define SET_ZLP_DATA 		9
#define GET_HIGH_FULL_SPEED 	10
#define SIG_SETUP		44

#define MTP_IOCTL_BASE			0x99
#define MTP_IO(nr)				_IO(MTP_IOCTL_BASE,nr)
#define MTP_IOR(nr,type)		_IOR(MTP_IOCTL_BASE,nr,type)
#define MTP_IOW(nr,type)		_IOW(MTP_IOCTL_BASE,nr,type)
#define MTP_IOWR(nr,type)		_IOWR(MTP_IOCTL_BASE,nr,type)
#define MTP_IOCTL_WRITE_ON_INTERRUPT_EP	MTP_IOW(0, char *)
#define MTP_IOCTL_GET_DEVICE_PHASE		MTP_IOR(1, unsigned short *)
#define MTP_IOCTL_SET_DEVICE_PHASE		MTP_IOW(2, unsigned short *)
#define MTP_IOCTL_GET_CANCEL_STATUS		MTP_IOR(3, char *)
#define MTP_IOCTL_SET_CANCEL_STATUS		MTP_IOW(4, char *)
#define MTP_IOCTL_CLEAR_FIFO			MTP_IOW(5, int)
#define MTP_IOCTL_SET_CONNECT_STATUS	MTP_IOW(6, int)
#define MTP_IOCTL_GET_CONNECT_STATUS	MTP_IOR(7, int)


/*PIMA15740-2000 spec*/
#define USB_PTPREQUEST_CANCELIO   0x64    /* Cancel request */
#define USB_PTPREQUEST_GETEVENT   0x65    /* Get extened event data */
#define USB_PTPREQUEST_RESET      0x66    /* Reset Device */
#define USB_PTPREQUEST_GETSTATUS  0x67    /* Get Device Status */
#define USB_PTPREQUEST_CANCELIO_SIZE 6
#define USB_PTPREQUEST_GETSTATUS_SIZE 12



int mtp_function_add(struct usb_configuration *c);
int mtp_function_config_changed(struct usb_composite_dev *cdev,	struct usb_configuration *c);
int mtp_enable(void);
void mtp_function_enable(int enable);

struct usb_mtp_ctrlrequest {
	struct usb_ctrlrequest	setup;
};

#endif /* __F_MTP_H */

