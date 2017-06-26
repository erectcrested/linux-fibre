/*
 * Framework for finding and configuring MAX24287 PHY device.
 *
 * Author: Boris-Ben Shapiro
 *
 * @ 01.04.15	-	revision 1
 * TODO: read GPIOS configuration from DT or platform data
 * TODO: read interface mode (RGMII/MII/GMII) from DT or platform data
 * TODO: read interrupts/poll configuration from DT or platform data
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#undef BASEX_IN_PHYDEVICE

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/cpsw.h>
#include <linux/sfp.h>
#include <linux/netdevice.h>
#include <linux/net_switch_config.h>

#include <linux/phylink.h>
#include "mdio-i2c.h"

#define MAX_FEATURES	(PHY_GBIT_FEATURES | SUPPORTED_Pause | SUPPORTED_FIBRE)
#define DEVICE_NAME					 "max24287"
#define GP_CONFIGURATION_MASK		   0x7		/* GPIOS mask */
#define PAGEBIT_MASK					0x3
#define MAX24287_IR					 (0x14)	/* Interrupt register */
#define IR_MASK						 0x2B00
/* Default interrupt mask: RFAULT + ALOS + RLOL + RLOS */

#define MAX24287_MINPAGE				0
#define MAX24287_MAXPAGE				3

/* gpio registers values */
#define GPVAL_HI				0x0
#define GPVAL_LOGIC0			0x1
#define GPVAL_LOGIC1			0x2
#define GPVAL_IROUT			 0x3
#define GPVAL_125OUT			0x4
#define GPVAL_PLLOUT			0x5
#define GPVAL_LINKST			0x6
#define GPVAL_OUTCRS			GPVAL_LINKST

#define PCSCR_BASEX			 (0x0010)
#define GMIICR_SPD1000		  (1 << 15)
#define GMIICR_SPD100		   (1 << 14)
#define GMII_CTRL_SPD_10		~(0xc000)
#define GMII_CTRL_SPD_100	   0x4000
#define GMII_CTRL_SPD_1000	  0x8000
#define GMII_CTRL_TBI		   0xc000
#define GMII_CTRL_DTE_DCE	   0x1000
#define GMII_CTRL_DDR		   0x800
#define GMII_CTRL_TXCLKEN	   0x400
#define GMII_CTRL_REFCLK_INV	0x8
#define GMII_CTRL_RLB		   0x1

/* LPA */
#define LPAX_LINK_FAIL		  0x1000
#define LPAX_OFF_LINE		   0x2000
#define LPAX_ANEG_ERR		   (LPAX_LINK_FAIL | LPAX_OFF_LINE)

/* pcs control register basex/sgmii control */
#define PCSCTRL_BASEX_COMMA_MASK		0x11
#define PCSCTRL_CFG_BASEX			   0x11
#define PCSCTRL_CFG_SGMII			   0x1

/* Advertisment basex/sgmii defines */
#define MAX24287_DEFAULT_BASEX_ADV		0xA0
#define MAX24287_DEFAULT_SGMII_ADV		0x1
#define MAX24287_LPA_SGMII_LKACK		  0x8C00

#define ANRX_BASEX_OFF_LINE			 (1 << 13)
#define ANRX_BASEX_LINK_FAIL			(1 << 12)
#define ANRX_BASEX_RF   (ANRX_BASEX_OFF_LINE | ANRX_BASEX_LINK_FAIL)

#define MAX24287_BASEX_LPAFULL			0x4020
#define MAX24287_SGMII_LPA1000			0x800
#define MAX24287_SGMII_LPA100			 0x400
#define MAX24287_SGMII_FDX				0x1000

#define PHY_PAGE0	   0
#define PHY_PAGE1	   1
#define PHY_PAGE2	   2
#define PHY_PAGE3	   3


static int max24287_config_advert(struct phy_device *);
static int max24287_config_aneg(struct phy_device *);
static int max24287_config_init(struct phy_device *phydev);
static int max24287_rev_b_reset(struct phy_device *);
static int _max24287_do_aneg(struct phy_device *phydev);

static inline int isbasex(struct phy_device *phydev)
{
	int pcs = phy_read(phydev, MII_PCSCTRL);

	return (pcs < 0 ? pcs : !!(pcs & PCSCR_BASEX));
}

static int max24287_open_page(struct phy_device *phydev, int page)
{
	int ret;
	u16 regval, oldpage;

	if (page > MAX24287_MAXPAGE || page < MAX24287_MINPAGE)
		return -EINVAL;

	ret = phy_read(phydev, MII_EXT_PAGE);
	if (ret < 0)
		return -ENXIO;

	regval = oldpage = (u16)((u16)ret & PAGEBIT_MASK);

	if (oldpage == page)
		return oldpage;

	regval |= (1 << 4);	/* This bit is required */
	regval &= ~PAGEBIT_MASK;
	regval |= page;
	ret = phy_write(phydev, MII_EXT_PAGE, (int)regval)
	if (ret < 0)
		return -ENXIO;

	return oldpage;
}

static int max24287_set_gpios(struct phy_device *phydev, u8 page, u16 reg,
										unsigned char bit, u8 configuration)
{
	int err, val, oldpage;

	/* Maximum allowed value is last register bit (16) minus one value (3)=13 */
	if (bit > 13)
		return -EOVERFLOW;
	if (configuration & ~(GP_CONFIGURATION_MASK))
		return -EOVERFLOW;

	oldpage = max24287_open_page(phydev, page);
	if (oldpage < 0) {
		dev_crit(&phydev->dev, "Error %d opening page %d", oldpage, page);
		return oldpage;
	}

	val = phy_read(phydev, reg);
	if (val < 0) {
		max24287_open_page(phydev, oldpage);
		return val;
	}

	val &= ~((1 << bit) | (1 << (bit+1)) | (1 << (bit+2)));

	val |= (configuration << bit);
	err = phy_write(phydev, reg, val);
	if (err < 0) {
		dev_err(&phydev->dev, "Error %d writing new GPIO configuration 0x%04x
												to 0x%02xn", err, val , reg);
		max24287_open_page(phydev, oldpage);
		return err;
	} else {
		dev_dbg(&phydev->dev, "%s wrote new GPIO configuration 0x%04x to %d\n",
													__func__, val, reg);
	}

	err = max24287_open_page(phydev, oldpage);
	if (err < 0)
		return err;

	return 0;
}

/* max24287_set_basex
 * Changes PCS link timer register
 */
static int max24287_set_basex(struct phy_device *phydev, bool basex)
{
	int cur = isbasex(phydev), pcs;

	if ((int)basex == cur)
		return 0;
	pcs = phy_read(phydev, MII_PCSCTRL);
	if (pcs < 0)
		return pcs;

	phydev->supported = MAX_FEATURES;

	if (basex) {
		phydev->supported = SUPPORTED_1000baseT_Full |
								SUPPORTED_FIBRE | SUPPORTED_Pause;
		phy_write(phydev, MII_ADVERTISE, MAX24287_DEFAULT_BASEX_ADV);
		pcs |= PCSCR_BASEX;
	} else {
		phydev->supported &= ~(SUPPORTED_FIBRE | SUPPORTED_Pause);
		phy_write(phydev, MII_ADVERTISE, MAX24287_DEFAULT_SGMII_ADV);
		pcs &= ~(PCSCR_BASEX);
	}
	phydev->advertising &= phydev->supported;

	pcs = phy_write(phydev, MII_PCSCTRL, pcs);
	if (pcs < 0)
		return pcs;

	return genphy_restart_aneg(phydev);
}

/* max24287_sysfs_set_basex
 * configures link timer to SGMII/1000base-x from userspace
 */
static ssize_t max24287_sysfs_set_basex(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct phy_device *phyd = to_phy_device(dev);
	bool bool_val = (bool)(!!simple_strtol(buf, NULL, 10));
	bool basex = (bool)isbasex(phyd);
	int res;

	dev_info(&phyd->dev, "Basex is %d, setting to %d\n",
										(int)basex, (int)bool_val);

	res = max24287_set_basex(phyd, bool_val);
	if (!res)
		return count;
	else
		return -EAGAIN;
}

/* max24287_sysfs_get_basex
 * SYSFS routine (per mdio address), returns 1 if port is in 1000basex mode
*/
static ssize_t max24287_sysfs_get_basex(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	struct phy_device			   *phyd = to_phy_device(dev);
	bool 							basex = (bool)isbasex(phyd);
	int								val = (basex == true) ? 1 : 0;

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR(conf_basex, 0644,
						max24287_sysfs_get_basex, max24287_sysfs_set_basex);

static int max24287_config_intr(struct phy_device *phydev)
{
	int	 val;
	int	ret;

	max24287_open_page(phydev, 0);
	val = phy_read(phydev, MAX24287_IR);
	if (val < 0)
		return val;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		val |= IR_MASK;
	else
		val &= ~(IR_MASK);

	ret = phy_write(phydev, MAX24287_IR, val);
	return ret;
}

static int max24287_ack_intr(struct phy_device *phydev)
{
	int	 val;

	val = phy_read(phydev, MII_EXT_PAGE);
	if (val < 0)
		return val;

	return 0;
}

/* max24287_gmiicr_set
 * Sets rx clock speed. see datasheet, register description GMIICR
 *
 * @argval struct phy_device	*phydev 	- phy device pointer
 * @argval int					speed		- speed
 *
 * @retval	- 0 success, 1 error
 */
static int max24287_gmiicr_set(struct phy_device *phydev, int speed)
{
	int	gmiicr;

	gmiicr = phy_read(phydev, MII_GMIICR);
	if (gmiicr < 0)
		return gmiicr;

	gmiicr &= ~(GMIICR_SPD100 | GMIICR_SPD1000);

	switch (speed) {
	case SPEED_1000:
		gmiicr |= GMIICR_SPD1000;
		break;
	case SPEED_100:
		gmiicr |= GMIICR_SPD100;
		break;
	case SPEED_10:
		break;
	default:
		dev_err(&phydev->dev, "%s unknown speed %d\n", __func__, speed);
		return -EINVAL;
	}
	return phy_write(phydev, MII_GMIICR, gmiicr);
}

/* _max24287_do_aneg
 * @phydev: target phy_device struct
 *
 * Description:	Setup phy according to basex aneg
 *
 * retval: 0 ok, <>0 error
 */
static int _max24287_do_aneg(struct phy_device *phydev)
{
	int				lpa, gmiicr;
	bool				basex = (bool)isbasex(phydev);

	lpa = phy_read(phydev, MII_LPA);
	if (lpa < 0)
		return lpa;

	#ifndef BASEX_IN_PHYDEVICE
	/* Autodetection in case that PHYLINK cannot set basex */
	if (basex == true && ((lpa & MAX24287_LPA_SGMII_LKACK) > 0x8000)) {
		dev_info(&phydev->dev, "%s Detected SGMII status. but link timer is
						for Base-X. Changing (0x%04x)\n", __func__, lpa);
		gmiicr = max24287_set_basex(phydev, false);
		if (gmiicr < 0)
			return gmiicr;
		else
			basex = false;
	} else if (basex == false &&
				((lpa & MAX24287_BASEX_LPAFULL) == MAX24287_BASEX_LPAFULL)) {
		dev_info(&phydev->dev, "%s Detected 1000BASE-X status. but link timer
						is for SGMII. Changing (0x%04x)\n", __func__, lpa);
		gmiicr = max24287_set_basex(phydev, true);
		if (gmiicr < 0)
			return gmiicr;
		else
			basex = true;
	}
	#endif
	phydev->speed = SPEED_10;
	phydev->duplex = DUPLEX_HALF;
	phydev->pause = phydev->asym_pause = 0;
	/* NOTE: we do not handle symmetric pause */

	if (basex) {
		if (!((phydev->supported & SUPPORTED_1000baseT_Full) &&
					(phydev->advertising & ADVERTISED_1000baseT_Full))) {
			dev_err(&phydev->dev, "1000BASE-X not supported\n");
			return 0;
		}

	/* Handle Remote fault first */
	if ((lpa & ANRX_BASEX_RF) == ANRX_BASEX_OFF_LINE) {
		dev_err(&phydev->dev, "%s 0x%02x: Partner off-line!\n", __func__, lpa);
		return 0;
	} else if ((lpa & ANRX_BASEX_RF) == ANRX_BASEX_LINK_FAIL) {
		dev_err(&phydev->dev, "%s: 0x%02x: Link Failure\n", __func__, lpa);
		return 0;
	} else if ((lpa & ANRX_BASEX_RF) ==  ANRX_BASEX_RF) {
		int p = !!(phydev->advertising & ADVERTISED_Pause);
		dev_err(&phydev->dev, "%s: 0x%02x: Auto-Negotiation Error.
							Toggling pause [%d]!\n", __func__, lpa, p);
		if (phydev->supported & SUPPORTED_Pause) {
			if (p)
				phydev->advertising &= ~ADVERTISED_Pause;
			else
				phydev->advertising |= ADVERTISED_Pause;
		}
		goto skip_pause;
	}

		/* Handle pause capability recieved from link partner */
	if (phydev->supported & SUPPORTED_Pause) {
		if (lpa & LPA_1000XPAUSE)
			phydev->advertising |= ADVERTISED_Pause;
		else
			phydev->advertising &= ~ADVERTISED_Pause;
	} else {
		phydev->advertising &= ~(ADVERTISED_Pause | ADVERTISED_Asym_Pause);
	}
	phydev->pause = !!(phydev->advertising & ADVERTISED_Pause);
skip_pause:
	gmiicr = !!(lpa & LPA_1000XFULL);
	if (gmiicr) {
		phydev->advertising |= ADVERTISED_1000baseT_Full;
		phydev->speed = SPEED_1000;
		phydev->duplex = DUPLEX_FULL;
		gmiicr = max24287_gmiicr_set(phydev, SPEED_1000);
		if (gmiicr != 0)
			return 1;
	}
	return 0;
	}
	/* Following is analyzing SGMII LPA page 34 in datasheet */
	if (lpa & MAX24287_SGMII_LPA1000) {
		if (((phydev->supported & SUPPORTED_1000baseT_Full) &&
			 (phydev->advertising & ADVERTISED_1000baseT_Full)) ||
				((phydev->supported & SUPPORTED_1000baseT_Half) &&
				 (phydev->advertising & ADVERTISED_1000baseT_Half))) {
					phydev->speed = SPEED_1000;
					gmiicr = max24287_gmiicr_set(phydev, SPEED_1000);
		} else {
			dev_err(&phydev->dev, "1000BASET not supported. 0x%08x\n", lpa);
		}
	} else if (lpa & MAX24287_SGMII_LPA100) {
		if (((phydev->supported & SUPPORTED_100baseT_Full) &&
			 (phydev->advertising & ADVERTISED_100baseT_Full)) ||
				((phydev->supported & SUPPORTED_100baseT_Half) &&
				 (phydev->advertising & ADVERTISED_100baseT_Half))) {
			phydev->speed = SPEED_100;
			gmiicr = max24287_gmiicr_set(phydev, SPEED_100);
		} else {
			dev_err(&phydev->dev, "100BASET not supported. 0x%08x\n", lpa);
		}
	} else {
		phydev->speed = SPEED_10;
		gmiicr = max24287_gmiicr_set(phydev, SPEED_10);
	}

	if (lpa & MAX24287_SGMII_FDX) {
		if (((phydev->supported & SUPPORTED_100baseT_Full) &&
			 (phydev->advertising & ADVERTISED_100baseT_Full)) ||
				((phydev->supported & SUPPORTED_1000baseT_Full) &&
				 (phydev->advertising & ADVERTISED_1000baseT_Full)))
		phydev->duplex = DUPLEX_FULL;
	}

	return gmiicr;
}

/* max24287_update_link
 * The link on the SGMII side appears as different bits in SGMII or 1000baseX
 * advertisement register
 */
static int max24287_update_link(struct phy_device *phydev)
{
	int lpa, basex = isbasex(phydev);

	lpa = genphy_update_link(phydev);
	if (lpa < 0)
		return lpa;
	else if (phydev->link == false)
		return 0;

	lpa = phy_read(phydev, MII_LPA);
	if (lpa < 0)
		return lpa;

	if (basex)
		phydev->link = (bool)(!!(lpa & 0x0020));
	else
		phydev->link = (bool)(!!(lpa & 0x8000));

	return 0;
}

static int max24287_read_status(struct phy_device *phydev)
{
		int err;

	err = max24287_update_link(phydev);
	if (err)
		return err;

	return _max24287_do_aneg(phydev);
}

static int max24287_config_advert(struct phy_device *phydev)
{
	u32 advertise;
	int orig_adv, err, adv = 0;
	int basex = isbasex(phydev);

	/* Only allow advertising what
	 * this PHY supports */
	phydev->advertising &= phydev->supported;
	advertise = phydev->advertising;
	orig_adv = phy_read(phydev, MII_ADVERTISE);

	if (orig_adv < 0)
		return orig_adv;

	if (basex) {
		if (advertise & ADVERTISED_1000baseT_Full)
			adv |= ADVERTISE_1000XFULL;
		else
			adv |= ANRX_BASEX_RF;   /* Remote fault */

		if (phydev->pause) {
			if (advertise & ADVERTISED_Pause)
				adv |= ADVERTISE_1000XPAUSE;
		}
	} else {
		adv = MAX24287_DEFAULT_SGMII_ADV;	/* Datasheet speicfies this value when using SGMII */
	}

	if (adv != orig_adv) {
		err = phy_write(phydev, MII_ADVERTISE, adv);
		if (err < 0)
			return err;
		return 1;
	}
	return 0;
}

static int max24287_config_aneg(struct phy_device *phydev)
{
	int 				err;

	err = max24287_config_advert(phydev);

	if (err < 0) {
		return err;
	} else if (err == 0) {
		int	ctl = phy_read(phydev, MII_BMCR);

		if (ctl < 0) {
			dev_err(&phydev->dev, "%s: error %d reading BMCR\n", __func__, ctl);
			return ctl;
		}

		if (!(ctl & BMCR_ANENABLE))
			err = 1;
	}

	if (err > 0)
		err = genphy_restart_aneg(phydev);

	return err;
}


/**
 * max24287_config_init - initial config of max24287
 * @phydev: target phy_device struct
 *
 * Description: Called upon interface initialization (ifconfig up and so)
 *
 * @retval:  0	- success
 * @retval: <0	- error
*/
static int max24287_config_init(struct phy_device *phydev)
{
	int val;
	u32 features = 0;

	#if 0
	features = MAX_FEATURES;
	max24287_gmiicr_set(phydev, SPEED_1000);
	max24287_set_basex(phydev, true);
	#endif
	val = phy_read(phydev, MII_BMSR);

	if (val < 0)
		return val;

	if (val & BMSR_ANEGCAPABLE)
		features |= SUPPORTED_Autoneg;

	features |= SUPPORTED_1000baseT_Full;
	features |= SUPPORTED_1000baseT_Half;
	features |= SUPPORTED_100baseT_Full;
	features |= SUPPORTED_100baseT_Half;
	features |= SUPPORTED_10baseT_Full;
	features |= SUPPORTED_10baseT_Half;
	features |= SUPPORTED_Pause;
	features |= SUPPORTED_FIBRE;

	phydev->supported = features;
	phydev->advertising = features;

	return 0;
}
/**
 * max24287_rev_b_reset
 * @phydev: target phydev to use
 *
 * Description: max24287 soft reset procedure as stated in datasheet page 39
*/
static int max24287_rev_b_reset(struct phy_device *phydev)
{
	int val, err = 0;

	dev_dbg(&phydev->dev, "About to perform device reset (rev B).");
	err = phy_read(phydev, MII_EXT_PAGE);
	if (err < 0)
		return err;
	val = 0x0012;
	err = phy_write(phydev, MII_EXT_PAGE, val);
	if (err < 0)
		return err;
	val = 0x4004;
	err = phy_write(phydev, MII_PTPCR1, val);
	if (err < 0)
		 return err;
	udelay(1000);
	val = 0x4000;
	err = phy_write(phydev, MII_PTPCR1, val);
	if (err < 0)
		return err;
	phy_write(phydev, MII_EXT_PAGE, 0x0010);
	genphy_reset(phydev);
	return 0;
}

/**
 * max24287_phy_probe
 * *
 * Description: Driver probe method for max24287 driver. max24287 has phy_id of
 * 0, and it's real id is stored in page1 ID register.
 *
 * @retval: 0 	= success
 * @retval: <0 	= error
*/
static int max24287_phy_probe(struct phy_device *phydev)
{
	int	err = 0, i;
	int gpio4to7_val[5] = {GPVAL_HI, GPVAL_HI, GPVAL_HI, GPVAL_HI, GPVAL_HI};
	/* All hi impedance. XXX: This needs to be passed from DT or board file */
	int gpos_andgpios[5] = {GPVAL_HI, GPVAL_HI, GPVAL_HI, GPVAL_HI, GPVAL_IROUT};
	/* GPO1-interrupt. XXX: This needs to be passed from DT or board file */
	u16 phy_id = 0, rev = 0;

	#if 0
	/* XXX: This needs to be passed from DT or board file */
	phydev->interface = PHY_INTERFACE_MODE_RGMII;
	#endif

	err = max24287_open_page(phydev, 1);
	if (err < 0)
		return -ENODEV;
	err = phy_read(phydev, 0x10);
	if (err < 0)
		return -EIO;

	phy_id = err & 0x0FFF;
	rev = (err & 0xF000) >> 12;

	dev_info(&phydev->dev, "Detected phy_id 0x%04x, revision 0x%01x\n",
															phy_id, rev);
	if (phy_id == 0xee0) {
		err = max24287_rev_b_reset(phydev);
		if (err)
			return err;
	} else if (phy_id != 0xedf) {
		return -ENODEV;
	}

	for (i = 0; i < 5; i++) {
		err = max24287_set_gpios(phydev, 1/*page1*/, 18, i*3, gpio4to7_val[i]);
		if (err < 0)
			dev_info(&phydev->dev, "Error %d setting bit %d in register 1.18",
																	err, i*3);
		err = max24287_set_gpios(phydev, 1/*page1*/, 17, i*3, gpos_andgpios[i])
		if (err < 0)
			dev_info(&phydev->dev, "Error %d setting bit %d in register 1.17",
																	err, i*3);
	}

	err = device_create_file(&phydev->dev, &dev_attr_conf_basex);
	if (err != 0)
		return err;

	return 0;
}

static void max24287_phy_remove(struct phy_device *phydev)
{
	device_remove_file(&phydev->dev, &dev_attr_conf_basex);
}


static struct phy_driver max24287_phy_driver = {
	.phy_id		 = 0x00000000,
	.phy_id_mask	= 0xffffffff,
	.name			= DEVICE_NAME,
	.features		= MAX_FEATURES,
	/* no assymetric pause */
	.flags		  =   0,
	.config_init	= max24287_config_init,
	.config_aneg	= max24287_config_aneg,
	.read_status	= max24287_read_status,
	.ack_interrupt	= max24287_ack_intr,
	.config_intr	= max24287_config_intr,
	#ifdef BASEX_IN_PHYDEVICE
	/* XXX: add a prototype in struct phy_device in include/linux/phy.h */
	.set_basex	= max24287_set_basex,
	#endif
	.probe		= max24287_phy_probe,
	.remove		= max24287_phy_remove,
	.driver		= {.owner = THIS_MODULE, },
};

static int __init max24287_init(void)
{
	int ret;

	pr_info("%s: All bugs added by Ben Shapiro\n", DEVICE_NAME);

	ret = phy_driver_register(&max24287_phy_driver);
	if (ret < 0)
		return ret;

	return 0;
}

static void __exit max24287_exit(void)
{
	phy_driver_unregister(&max24287_phy_driver);
}

module_init(max24287_init);
module_exit(max24287_exit);

/* XXX: max24287 has all 0's in PHYID1 and PHYID2. so we register 0 as
 * mdio_device phy_id then at probe function, we can read page1 ID registers
 * the real ID
 */
static struct mdio_device_id __maybe_unused max24287_tbl[] = {
		{ 0x00000000, 0xffffffff },
	{ }
};
MODULE_DEVICE_TABLE(mdio, max24287_tbl);

MODULE_DESCRIPTION("max24287 PHY driver");
MODULE_AUTHOR("Boris-Ben Shapiro");
MODULE_LICENSE("GPL");
