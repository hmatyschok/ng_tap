/*-
 * Copyright (c) 2015, Adrian Chadd <adrian@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/*
 * Copyright (c) 2018 Henning Matyschok
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
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#include <sys/cdefs.h>
__FBSDID("$FreeBSD: head/sys/mips/atheros/ar71xx_macaddr.c 280798 2015-03-28 23:40:29Z adrian $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/types.h>
#include <sys/libkern.h>

#include <net/ethernet.h>

#include <mips/atheros/ar71xx_macaddr.h>

/*
 * Some boards don't have a separate MAC address for each individual
 * device on-board, but instead need to derive them from a single MAC
 * address stored somewhere.
 */
uint8_t ar71xx_board_mac_addr[ETHER_ADDR_LEN];

/*
 * Initialise a MAC address 'dst' from a MAC address 'src'.
 *
 * 'offset' is added to the low three bytes to allow for sequential
 * MAC addresses to be derived from a single one.
 *
 * 'is_local' is whether this 'dst' should be made a local MAC address.
 *
 * Returns 0 if it was successfully initialised, -1 on error.
 */
int
ar71xx_mac_addr_init(unsigned char *dst, const unsigned char *src,
    int offset, int is_local)
{
	int t;

	if (dst == NULL || src == NULL)
		return (-1);

	if (ETHER_IS_MULTICAST(src) != 0)
		return (-1);

	t = (((uint32_t) src[3]) << 16)
	    + (((uint32_t) src[4]) << 8)
	    + ((uint32_t) src[5]);

	/* Note: this is handles both positive and negative offsets */
	t += offset;

	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
	dst[3] = (t >> 16) & 0xff;
	dst[4] = (t >> 8) & 0xff;
	dst[5] = t & 0xff;

	if (is_local)
		dst[0] |= 0x02;

	/* Everything's okay */
	return (0);
}

/*
 * Initialise a random MAC address for use by if_arge.c and whatever
 * else requires it.
 *
 * Returns 0 on success, -1 on error.
 */
int
ar71xx_mac_addr_random_init(unsigned char *dst)
{
	uint32_t rnd;

	rnd = arc4random();

	dst[0] = 'b';
	dst[1] = 's';
	dst[2] = 'd';
	dst[3] = (rnd >> 24) & 0xff;
	dst[4] = (rnd >> 16) & 0xff;
	dst[5] = (rnd >> 8) & 0xff;

	return (0);
}

/*
 * Initialize MAC Address by hints mechanism.
 * 
 * Returns 1 if MAC Address exists, 0 if not.
 */
int
ar71xx_mac_addr_hint_init(device_t dev, unsigned char *addr)
{
	char 	dev_id[32];
	char * 	mac_addr;
	uint32_t tmp_addr[ETHER_ADDR_LEN];
	int 	count;
	int 	i;
	int 	local_mac;
	
	(void)snprintf(dev_id, 32, "hint.%s.%d.macaddr", 
	    device_get_name(dev), device_get_unit(dev));
	
	if ((mac_addr = kern_getenv(dev_id)) != NULL) {
		/* Have a MAC address; should use it. */
		device_printf(dev, "Overriding MAC address "
			"from environment: '%s'\n", mac_addr);
		
		/* Extract out the MAC address. */
		count = sscanf(mac_addr, 
			"%x%*c%x%*c%x%*c%x%*c%x%*c%x",
				&tmp_addr[0], &tmp_addr[1],
				&tmp_addr[2], &tmp_addr[3],
				&tmp_addr[4], &tmp_addr[5]);

		/* Valid! */
		if (count == ETHER_ADDR_LEN) {
			for (i = 0; i < count; i++)
				addr[i] = tmp_addr[i];
		
			local_mac = 1;
		} else 
			local_mac = 0;

		/* Done! */
		freeenv(mac_addr);
		mac_addr = NULL;
	} else 
		local_mac = 0;

	return (local_mac);
}

/*
 * Initialize MAC Address by reading EEPROM.
 * 
 * Some units (eg the TP-Link WR-1043ND) do not have a convenient
 * EEPROM location to read the ethernet MAC address from.
 * OpenWRT simply snaffles it from a fixed location.
 *
 * Since multiple units seem to use this feature, include
 * a method of setting the MAC address based on an flash location
 * in CPU address space.
 *
 * Some vendors have decided to store the mac address as a literal
 * string of 18 characters in xx:xx:xx:xx:xx:xx format instead of
 * an array of numbers.  Expose a hint to turn on this conversion
 * feature via strtol().
 * 
 * Returns 1 if MAC Address exists, 0 if not.
 */
int 
ar71xx_mac_addr_eeprom_init(device_t dev, unsigned char *addr)
{
	const char *name;
	int 	unit;
	long 	eeprom_mac_addr;
	const char *mac_addr;
	int		readascii; 
	int 	i;
	int  	local_mac;
	 
	name = device_get_name(dev); 
	unit = device_get_unit(dev);
	eepprom_mac_addr = 0;
	 
	if (resource_long_value(name, unit, 
	    "eeprommac", &eeprom_mac_addr) == 0) {
		device_printf(dev, "Overriding MAC from EEPROM\n");
		
		mac_addr = (const char *)MIPS_PHYS_TO_KSEG1(eeprom_mac_addr);
		readascii = 0;
		
		if (resource_int_value(name, unit, 
		    "readascii", &readascii) == 0) {
			device_printf(dev, "Vendor stores MAC in ASCII format\n");

			for (i = 0; i < ETHER_ADDR_LEN; i++) {
				addr[i] = strtol(&(mac_addr[i*3]), NULL, 16);
			}
		} else {
			for (i = 0; i < ETHER_ADDR_LEN; i++) {
				addr[i] = mac_addr[i];
			}
		}
		local_mac = 1;
	} else
		local_mac = 0;

	return (local_mac);
}
