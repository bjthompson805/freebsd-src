/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Userland control interface of the Apple N1 Wi-Fi function (apple_n1),
 * /dev/n1wifi. A copy of this header is used by the n1wifi tool; the two
 * must stay identical (N1WIFI_VERSION is checked by the tool).
 *
 * The firmware does 802.11 and the WPA2 4-way handshake itself, from a PMK:
 * userland computes PMK = PBKDF2-HMAC-SHA1(passphrase, SSID, 4096, 32) and
 * passes it with N1WIFI_JOIN. The network interface is n1w0 (Ethernet).
 */

#ifndef _ARM64_APPLE_APPLE_N1_WIFI_H_
#define	_ARM64_APPLE_APPLE_N1_WIFI_H_

#include <sys/types.h>
#include <sys/ioccom.h>

#define	N1WIFI_VERSION		1
#define	N1WIFI_MAXBSS		64

/* n1wifi_bss.security */
#define	N1WIFI_SEC_PRIVACY	0x0001	/* capability privacy bit */
#define	N1WIFI_SEC_RSN		0x0002	/* RSN (WPA2/WPA3) element present */
#define	N1WIFI_SEC_WPA1		0x0004	/* WPA (vendor) element present */
#define	N1WIFI_SEC_PSK		0x0010	/* RSN AKM 2, PSK */
#define	N1WIFI_SEC_PSK256	0x0020	/* RSN AKM 6, PSK-SHA256 */
#define	N1WIFI_SEC_SAE		0x0040	/* RSN AKM 8, SAE (WPA3-Personal) */
#define	N1WIFI_SEC_8021X	0x0080	/* RSN AKM 1/5, 802.1X (Enterprise) */
#define	N1WIFI_SEC_CCMP		0x0100	/* group and a pairwise cipher CCMP */
#define	N1WIFI_SEC_PMF_REQ	0x0200	/* RSN capabilities: MFP required */
#define	N1WIFI_SEC_JOINABLE	0x8000	/* this driver can join it */

struct n1wifi_bss {
	uint8_t		bssid[6];
	uint8_t		ssid_len;	/* 0: hidden */
	uint8_t		channel;
	uint8_t		ssid[32];	/* not NUL terminated */
	int8_t		rssi;		/* dBm */
	uint8_t		band;		/* 0 2.4 GHz, 1 5 GHz */
	uint16_t	security;	/* N1WIFI_SEC_* */
	uint16_t	rsn_caps;
	uint16_t	reserved;
	uint32_t	scan_gen;	/* scan in which it was last seen */
};

struct n1wifi_list {
	uint32_t	count;		/* out: entries valid */
	uint32_t	scan_gen;	/* out: last completed scan */
	struct n1wifi_bss bss[N1WIFI_MAXBSS];
};

/* n1wifi_status.flags */
#define	N1WIFI_F_UP		0x0001	/* firmware running (alpha stage 2) */
#define	N1WIFI_F_RADIO		0x0002	/* radio enabled (see N1WIFI_RADIO) */
#define	N1WIFI_F_SCANNING	0x0004
#define	N1WIFI_F_JOINING	0x0008
#define	N1WIFI_F_CONNECTED	0x0010
#define	N1WIFI_F_IFNET		0x0020	/* n1w0 exists */

struct n1wifi_status {
	uint32_t	version;	/* N1WIFI_VERSION */
	uint32_t	flags;		/* N1WIFI_F_* */
	uint32_t	scan_gen;	/* completed scans */
	uint32_t	join_gen;	/* completed join attempts */
	int32_t		join_error;	/* errno of the last join, 0 ok */
	uint16_t	join_status;	/* firmware status of the last join */
	uint8_t		mac[6];
	/* The current (or last attempted) network */
	uint8_t		ssid[32];
	uint8_t		ssid_len;
	uint8_t		bssid[6];
	uint8_t		channel;
	uint8_t		band;
	int8_t		rssi;		/* from the last scan that saw it */
	uint16_t	reserved;
};

/* n1wifi_join.flags */
#define	N1WIFI_J_BSSID		0x01	/* join this BSSID, not the best */

struct n1wifi_join {
	uint8_t		ssid[32];
	uint8_t		ssid_len;
	uint8_t		flags;
	uint8_t		bssid[6];
	uint8_t		pmk[32];	/* PBKDF2-HMAC-SHA1(pass, ssid, 4096) */
};

/* Anyone may open the device; see the security notes on each command. */
#define	N1WIFI_STATUS	_IOR('N', 1, struct n1wifi_status)	/* anyone */
#define	N1WIFI_LIST	_IOR('N', 2, struct n1wifi_list)	/* anyone */
#define	N1WIFI_SCAN	_IO('N', 3)		/* root, wheel, operator */
#define	N1WIFI_JOIN	_IOW('N', 4, struct n1wifi_join)	/* root */
#define	N1WIFI_DISCONNECT _IO('N', 5)				/* root */
#define	N1WIFI_RADIO	_IOW('N', 6, int)			/* root */

#endif /* _ARM64_APPLE_APPLE_N1_WIFI_H_ */
