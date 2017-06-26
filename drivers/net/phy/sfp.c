#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#ifdef CONFIG_OF
#include <linux/of_net.h>
#endif
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>
#include <linux/sfp.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <net/net_namespace.h>
#include "mdio-i2c.h"
#include "swphy.h"
#include <linux/elspec_def.h>

enum {
	GPIO_MODDEF0,
	GPIO_LOS,
	GPIO_TX_FAULT,
	GPIO_TX_DISABLE,
	GPIO_RATE_SELECT,
	GPIO_MAX,

	SFP_F_PRESENT = BIT(GPIO_MODDEF0),
	SFP_F_LOS = BIT(GPIO_LOS),
	SFP_F_TX_FAULT = BIT(GPIO_TX_FAULT),
	SFP_F_TX_DISABLE = BIT(GPIO_TX_DISABLE),
	SFP_F_RATE_SELECT = BIT(GPIO_RATE_SELECT),

	SFP_E_INSERT = 0,
	SFP_E_REMOVE,
	SFP_E_DEV_DOWN,
	SFP_E_DEV_UP,
	SFP_E_TX_FAULT,
	SFP_E_TX_CLEAR,
	SFP_E_LOS_HIGH,
	SFP_E_LOS_LOW,
	SFP_E_TIMEOUT,

	SFP_MOD_EMPTY = 0,
	SFP_MOD_PROBE,
	SFP_MOD_PRESENT,
	SFP_MOD_ERROR,

	SFP_DEV_DOWN = 0,
	SFP_DEV_UP,

	SFP_S_DOWN = 0,
	SFP_S_INIT,
	SFP_S_WAIT_LOS,
	SFP_S_LINK_UP,
	SFP_S_TX_FAULT,
	SFP_S_REINIT,
	SFP_S_TX_DISABLE,
};

static const char *gpio_of_names[] = {
	"moddef0",
	"los",
	"tx-fault",
	"tx-disable",
	"rate-select",
};

static const enum gpiod_flags gpio_flags[] = {
	GPIOD_IN,
	GPIOD_IN,
	GPIOD_IN,
	GPIOD_ASIS,
	GPIOD_ASIS,
};

#define T_INIT_JIFFIES	msecs_to_jiffies(300)
#define T_RESET_US	10
#define T_FAULT_RECOVER	msecs_to_jiffies(1000)

/* SFP module presence detection is poor: the three MOD DEF signals are
 * the same length on the PCB, which means it's possible for MOD DEF 0 to
 * connect before the I2C bus on MOD DEF 1/2.  Try to work around this
 * design bug by waiting 50ms before probing, and then retry every 250ms.
 */
#define T_PROBE_INIT	msecs_to_jiffies(50)
#define T_PROBE_RETRY	msecs_to_jiffies(250)

/*
 * SFP modules appear to always have their PHY configured for bus address
 * 0x56 (which with mdio-i2c, translates to a PHY address of 22).
 */
#define SFP_PHY_ADDR	22

/*
 * Give this long for the PHY to reset.
 */
#define T_PHY_RESET_MS	50

static DEFINE_MUTEX(sfp_mutex);

struct sfp {
	struct device *dev;
	struct i2c_adapter *i2c;
	struct mii_bus *i2c_mii;
	struct net_device *ndev;
	struct phylink *phylink;
	struct phy_device *mod_phy;

	unsigned int (*get_state)(struct sfp *);
	void (*set_state)(struct sfp *, unsigned int);
	int (*read)(struct sfp *, bool, u8, void *, size_t);

	#if 0
	#ifdef CONFIG_OF
	struct gpio_desc *gpio[GPIO_MAX];
	#endif
	#else
	int		gpio[GPIO_MAX];
	#endif


	unsigned int state;
	struct delayed_work poll;
	struct delayed_work timeout;
	struct mutex sm_mutex;
	unsigned char sm_mod_state;
	unsigned char sm_dev_state;
	unsigned short sm_state;
	unsigned int sm_retries;

	unsigned int port;

	struct sfp_eeprom_id id;

	struct notifier_block netdev_nb;
};

static unsigned long poll_jiffies;

static int sfp_scan_phy_fixups(struct sfp *sfp, bool remove);
static int finiar_fclf_phy_fixup(struct phy_device *phy);

struct sfp_fixup {
	char vendor[17];
	char pn[17];
	char rev[5];
	int (*run)(struct phy_device *phy);
};

static struct sfp_fixup known_fixups[] = {
 { "FINISAR CORP.", "FCLF-8521-3", "A", finiar_fclf_phy_fixup},
};

static bool sfp_needs_fixup(struct sfp_eeprom_base *data, struct sfp_fixup *fix)
{
	if (strncasecmp(data->vendor_name, fix->vendor, strlen(fix->vendor)))
		return false;
	else if (strncasecmp(data->vendor_pn, fix->pn, strlen(fix->pn)))
		return false;
	else if (strncasecmp(data->vendor_rev, fix->rev, strlen(fix->rev)))
		return false;
	return true;
}

static int sfp_scan_phy_fixups(struct sfp *sfp, bool remove)
{
	int i;
	struct phy_device *phy = sfp->mod_phy;

	if (!phy) {
		dev_err(sfp->dev, "%s Error scanning for fixups on nonexistent phy\n", __func__);
		return -ENODEV;
	}

	for (i = 0; i < (sizeof(known_fixups) / sizeof(struct sfp_fixup)); i++) {
		if (sfp_needs_fixup(&(sfp->id.base), &(known_fixups[i]))) {
			int err;

			dev_info(sfp->dev, "SFP %s %s needs %s fixup (phy 0%x)\n", known_fixups[i].vendor, known_fixups[i].pn,
												(remove) ? "remove" : "add", phy->phy_id);
			if (!remove)
				err = phy_register_fixup_for_uid(phy->phy_id, PHY_ANY_UID, known_fixups[i].run);
			else
				err = phy_remove_fixup_for_uid(phy->phy_id, PHY_ANY_UID);

			if (err) {
				dev_err(sfp->dev, "FIXUP %s Failed. %d\n", (remove) ? "remove" : "add", err);
				return err;
			}
			break;
		}
	}
	return 0;
}

static inline char *mod_state_to_str(unsigned char state)
{
	switch(state) {
	case SFP_MOD_EMPTY:
		return "EMPTY";
	case SFP_MOD_PROBE:
		return "PROBE";
	case  SFP_MOD_PRESENT:
		return "PRESENT";
	case SFP_MOD_ERROR:
		return "ERROR";
	default:
		return "UNKNOWN";
	}
}

static inline char *dev_state_to_str(unsigned char state)
{
	switch(state) {
	case SFP_DEV_DOWN:
		return "DOWN";
	case SFP_DEV_UP:
		return "UP";
	default:
		return "UNKNOWN";
	}
}

static inline char *main_sm_to_str(unsigned char state)
{
	switch(state) {
	case SFP_S_DOWN:
		return "DOWN";
	case SFP_S_INIT:
		return "INITIALIZING";
	case SFP_S_WAIT_LOS:
		return "LINK UP";
	case SFP_S_LINK_UP:
		return "Loss-Of-Signal";
	case SFP_S_TX_FAULT:
		return "TX FAULT";
	case SFP_S_REINIT:
		return "REINIT";
	case SFP_S_TX_DISABLE:
		return "TX-DISABLE";
	default:
		return "UNKNOWN";
	}
}

static inline char *event_to_str(int event)
{
	switch(event) {
	case SFP_E_INSERT:
		return "INSERT";
	case SFP_E_REMOVE:
		return "REMOVE";
	case SFP_E_DEV_DOWN:
		return "DEV DOWN";
	case SFP_E_DEV_UP:
		return "DEV UP";
	case SFP_E_TX_FAULT:
		return "TX FAULT";
	case SFP_E_TX_CLEAR:
		return "TX CLEAR";
	case SFP_E_LOS_HIGH:
		return "LOS HIGH";
	case SFP_E_LOS_LOW:
		return "LOS LOW";
	case SFP_E_TIMEOUT:
		return "TIMEOUT";
	default:
		return "UNKNOWN";
	}
}

/* NOTE: Boris-Ben Shapiro (2016-08-01): Note our platform moddef0 is active low, so revert the result for it */
static unsigned int sfp_gpio_get_state(struct sfp *sfp)
{
	unsigned int i, state, v;

	/* XXX:*/
	for (i = state = 0; i < 1; i++) {
		if (!sfp->gpio[i] || gpio_flags[i] != GPIOD_IN)
			continue;
		v = gpio_get_value_cansleep(sfp->gpio[i]);
		if (!v)
			state |= BIT(i);
	}


	for (; i < GPIO_MAX; i++) {
		if (gpio_flags[i] != GPIOD_IN || !sfp->gpio[i])
			continue;

		v = gpio_get_value_cansleep(sfp->gpio[i]);
		if (v)
			state |= BIT(i);
	}

	return state;
}

static void sfp_gpio_set_state(struct sfp *sfp, unsigned int state)
{
	 if (state & SFP_F_PRESENT) {
		/* If the module is present, drive the signals */
		if (sfp->gpio[GPIO_TX_DISABLE])
			gpio_direction_output(sfp->gpio[GPIO_TX_DISABLE],
						!!(state & SFP_F_TX_DISABLE));
		if (state & SFP_F_RATE_SELECT)
			gpio_direction_output(sfp->gpio[GPIO_RATE_SELECT],
						!!(state & SFP_F_RATE_SELECT));
	} else {
		/* Otherwise, let them float to the pull-ups */
		if (sfp->gpio[GPIO_TX_DISABLE])
			gpio_direction_input(sfp->gpio[GPIO_TX_DISABLE]);
		if (state & SFP_F_RATE_SELECT)
			gpio_direction_input(sfp->gpio[GPIO_RATE_SELECT]);
	}
}

static int sfp__i2c_read(struct i2c_adapter *i2c, u8 bus_addr, u8 dev_addr,
	void *buf, size_t len)
{
	struct i2c_msg msgs[2];
	//struct i2c_client *dummy;
	int ret;
	#if 0
	u8 *reversed_data = kzalloc(len, GFP_KERNEL), *byte_buf = buf;
	s32 res;

	if (!reversed_data) {
		printk(KERN_EMERG "%s: No mem?!\n", __func__);
		return -ENOMEM;
	}
	if (!i2c) {
		printk(KERN_ERR "%s-%s: No i2c adap!\n", __FILE__, __func__);
		kfree(reversed_data);
		return -ENODEV;
	}

	if (!try_module_get(i2c->owner)) {
		printk(KERN_ERR "%s-%s: Failed getting i2c!\n", __FILE__, __func__);
		goto direct;
	}
	get_device(&i2c->dev);
	dummy = i2c_new_dummy(i2c, bus_addr);
	if (!dummy) {
		dev_err(&i2c->dev, "%s couldn't register dummy on 0x%02x\n", __func__, bus_addr);
		put_device(&i2c->dev);
		module_put(i2c->owner);
		goto direct;
	}
	if (!i2c_use_client(dummy)) {
		dev_err(&i2c->dev, "%s use dummy i2c client\n", __func__);
		i2c_unregister_device(dummy);
		put_device(&i2c->dev);
		module_put(i2c->owner);
		goto direct;
	}
	res = i2c_smbus_read_i2c_block_data(dummy, dev_addr, (u8)len, reversed_data);
	i2c_unregister_device(dummy);
	put_device(&i2c->dev);
	module_put(i2c->owner);

	sfpd("Dummy on bus addr 0x%02x read reg 0x%02x len %d(?==?) res (%d) ", bus_addr, dev_addr, (u8)len, res);
	if (res < 0) {
		dev_err(&i2c->dev, "Error %d reading reg 0x%02x", res, dev_addr);
		goto direct;
	}
	for (ret = 0; ret < res; ret++) 
		memcpy(&byte_buf[ret], &reversed_data[ret], 1);

	kfree(reversed_data);
	return (int)len;

	/* Original Code */
direct:
	kfree(reversed_data);
	#else
	msgs[0].addr = bus_addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &dev_addr;
	msgs[1].addr = bus_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = buf;

	ret = i2c_transfer(i2c, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;

	return ret == ARRAY_SIZE(msgs) ? len : 0;
	#endif
}

static int sfp_i2c_read(struct sfp *sfp, bool a2, u8 addr, void *buf,
	size_t len)
{
	return sfp__i2c_read(sfp->i2c, a2 ? 0x51 : 0x50, addr, buf, len);
}

static int sfp_i2c_configure(struct sfp *sfp, struct i2c_adapter *i2c)
{
	struct mii_bus *i2c_mii;
	u32 functionality;
	int ret;

	if (!i2c_check_functionality(i2c, I2C_FUNC_I2C))
		return -EINVAL;

	functionality = i2c_get_functionality(i2c);
	netdev_dbg(sfp->ndev, "adapter (%s) functionality 0x%08x\n", i2c->name, functionality);
	sfp->i2c = i2c;
	sfp->read = sfp_i2c_read;

	i2c_mii = mdio_i2c_alloc(sfp->dev, i2c);
	if (IS_ERR(i2c_mii))
		return PTR_ERR(i2c_mii);

	i2c_mii->name = "SFP I2C Bus";
	i2c_mii->phy_mask = ~0;

	ret = mdiobus_register(i2c_mii);
	if (ret < 0) {
		dev_err(&i2c->dev, "Error %d registering i2c mdiobus\n", ret);
		mdiobus_free(i2c_mii);
		return ret;
	}

	sfp->i2c_mii = i2c_mii;

	return 0;
}


/* Interface */
static unsigned int sfp_get_state(struct sfp *sfp)
{
	return sfp->get_state(sfp);
}

static void sfp_set_state(struct sfp *sfp, unsigned int state)
{
	sfp->set_state(sfp, state);
}

static int sfp_read(struct sfp *sfp, bool a2, u8 addr, void *buf, size_t len)
{
	return sfp->read(sfp, a2, addr, buf, len);
}

static unsigned int sfp_check(void *buf, size_t len)
{
	u8 *p, check;

	for (p = buf, check = 0; len; p++, len--)
		check += *p;

	return check;
}

static const char *sfp_link_len(char *buf, size_t size, unsigned int length,
	unsigned int multiplier)
{
	if (length == 0)
		return "unsupported/unspecified";

	if (length == 255) {
		*buf++ = '>';
		size -= 1;
		length -= 1;
	}

	length *= multiplier;

	if (length >= 1000)
		snprintf(buf, size, "%u.%0*ukm",
			length / 1000,
			multiplier > 100 ? 1 :
			multiplier > 10 ? 2 : 3,
			length % 1000);
	else
		snprintf(buf, size, "%um", length);

	return buf;
}

struct bitfield {
	unsigned int mask;
	unsigned int val;
	const char *str;
};

static const struct bitfield sfp_options[] = {
	{
		.mask = SFP_OPTIONS_HIGH_POWER_LEVEL,
		.val = SFP_OPTIONS_HIGH_POWER_LEVEL,
		.str = "hpl",
	}, {
		.mask = SFP_OPTIONS_PAGING_A2,
		.val = SFP_OPTIONS_PAGING_A2,
		.str = "paginga2",
	}, {
		.mask = SFP_OPTIONS_RETIMER,
		.val = SFP_OPTIONS_RETIMER,
		.str = "retimer",
	}, {
		.mask = SFP_OPTIONS_COOLED_XCVR,
		.val = SFP_OPTIONS_COOLED_XCVR,
		.str = "cooled",
	}, {
		.mask = SFP_OPTIONS_POWER_DECL,
		.val = SFP_OPTIONS_POWER_DECL,
		.str = "powerdecl",
	}, {
		.mask = SFP_OPTIONS_RX_LINEAR_OUT,
		.val = SFP_OPTIONS_RX_LINEAR_OUT,
		.str = "rxlinear",
	}, {
		.mask = SFP_OPTIONS_RX_DECISION_THRESH,
		.val = SFP_OPTIONS_RX_DECISION_THRESH,
		.str = "rxthresh",
	}, {
		.mask = SFP_OPTIONS_TUNABLE_TX,
		.val = SFP_OPTIONS_TUNABLE_TX,
		.str = "tunabletx",
	}, {
		.mask = SFP_OPTIONS_RATE_SELECT,
		.val = SFP_OPTIONS_RATE_SELECT,
		.str = "ratesel",
	}, {
		.mask = SFP_OPTIONS_TX_DISABLE,
		.val = SFP_OPTIONS_TX_DISABLE,
		.str = "txdisable",
	}, {
		.mask = SFP_OPTIONS_TX_FAULT,
		.val = SFP_OPTIONS_TX_FAULT,
		.str = "txfault",
	}, {
		.mask = SFP_OPTIONS_LOS_INVERTED,
		.val = SFP_OPTIONS_LOS_INVERTED,
		.str = "los-",
	}, {
		.mask = SFP_OPTIONS_LOS_NORMAL,
		.val = SFP_OPTIONS_LOS_NORMAL,
		.str = "los+",
	}, { }
};

static const struct bitfield diagmon[] = {
	{
		.mask = SFP_DIAGMON_DDM,
		.val = SFP_DIAGMON_DDM,
		.str = "ddm",
	}, {
		.mask = SFP_DIAGMON_INT_CAL,
		.val = SFP_DIAGMON_INT_CAL,
		.str = "intcal",
	}, {
		.mask = SFP_DIAGMON_EXT_CAL,
		.val = SFP_DIAGMON_EXT_CAL,
		.str = "extcal",
	}, {
		.mask = SFP_DIAGMON_RXPWR_AVG,
		.val = SFP_DIAGMON_RXPWR_AVG,
		.str = "rxpwravg",
	}, { }
};

static const char *sfp_bitfield(char *out, size_t outsz, const struct bitfield *bits, unsigned int val)
{
	char *p = out;
	int n;

	*p = '\0';
	while (bits->mask) {
		if ((val & bits->mask) == bits->val) {
			n = snprintf(p, outsz, "%s%s",
					 out != p ? ", " : "",
					 bits->str);
			if (n == outsz)
				break;
			p += n;
			outsz -= n;
		}
		bits++;
	}

	return out;
}

static const char *sfp_connector(unsigned int connector)
{
	switch (connector) {
	case SFP_CONNECTOR_UNSPEC:
		return "unknown/unspecified";
	case SFP_CONNECTOR_FIBERJACK:
		return "Fiberjack";
	case SFP_CONNECTOR_LC:
		return "LC";
	case SFP_CONNECTOR_MT_RJ:
		return "MT-RJ";
	case SFP_CONNECTOR_MU:
		return "MU";
	case SFP_CONNECTOR_SG:
		return "SG";
	case SFP_CONNECTOR_OPTICAL_PIGTAIL:
		return "Optical pigtail";
	case SFP_CONNECTOR_HSSDC_II:
		return "HSSDC II";
	case SFP_CONNECTOR_COPPER_PIGTAIL:
		return "Copper pigtail";
	default:
		return "unknown";
	}
}

static const char *sfp_encoding(unsigned int encoding)
{
	switch (encoding) {
	case SFP_ENCODING_UNSPEC:
		return "unspecified";
	case SFP_ENCODING_8B10B:
		return "8b10b";
	case SFP_ENCODING_4B5B:
		return "4b5b";
	case SFP_ENCODING_NRZ:
		return "NRZ";
	case SFP_ENCODING_MANCHESTER:
		return "MANCHESTER";
	default:
		return "unknown";
	}
}


/* Helpers */
static void sfp_module_tx_disable(struct sfp *sfp)
{
	netdev_dbg(sfp->ndev, "disabling sfp state");
	sfp->state |= SFP_F_TX_DISABLE;
	sfp_set_state(sfp, sfp->state);
}

static void sfp_module_tx_enable(struct sfp *sfp)
{
	netdev_dbg(sfp->ndev, "Enabling sfp state");
	sfp->state &= ~SFP_F_TX_DISABLE;
	sfp_set_state(sfp, sfp->state);
}

static void sfp_module_tx_fault_reset(struct sfp *sfp)
{
	unsigned int state = sfp->state;

	if (state & SFP_F_TX_DISABLE)
		return;

	sfp_set_state(sfp, state | SFP_F_TX_DISABLE);

	udelay(T_RESET_US);

	sfp_set_state(sfp, state);
}

/* SFP state machine */
static void sfp_sm_set_timer(struct sfp *sfp, unsigned int timeout)
{
	if (timeout)
	#ifdef NOW_SUPPORTED
		mod_delayed_work(system_power_efficient_wq, &sfp->timeout,
				 timeout);
	#else
//		queue_delayed_work(system_wq, &sfp->timeout, timeout);
		schedule_delayed_work(&sfp->timeout, timeout);
	#endif
	else
		cancel_delayed_work(&sfp->timeout);
}

static void sfp_sm_next(struct sfp *sfp, unsigned int state,
			unsigned int timeout)
{
	sfp->sm_state = state;
	sfp_sm_set_timer(sfp, timeout);
}

static void sfp_sm_ins_next(struct sfp *sfp, unsigned int state, unsigned int timeout)
{
	sfp->sm_mod_state = state;
	sfp_sm_set_timer(sfp, timeout);
}

static void sfp_sm_phy_detach(struct sfp *sfp)
{
	sfp_scan_phy_fixups(sfp, true);
	phy_stop(sfp->mod_phy);
	if (sfp->phylink)
		phylink_disconnect_phy(sfp->phylink);
	phy_device_remove(sfp->mod_phy);
	phy_device_free(sfp->mod_phy);
	sfp->mod_phy = NULL;
}

static void sfp_sm_probe_phy(struct sfp *sfp)
{
	struct phy_device *phy;
	int err;

	msleep(T_PHY_RESET_MS);

	netdev_dbg(sfp->ndev, "Will scan mdio bus");
	phy = mdiobus_scan(sfp->i2c_mii, SFP_PHY_ADDR);
	if (IS_ERR(phy)) {
		dev_err(sfp->dev, "mdiobus scan returned %ld\n", PTR_ERR(phy));
		return;
	}
	if (!phy) {
		dev_err(sfp->dev, "no PHY detected\n");
		return;
	}
	/* NOTE: Boris-Ben Shapiro (2017-04-20): Because of 3-way negotation we want to block the MEDIA phy of autonegotiating
	 * as this will happen anyway by default. And if the non-default is chosen, than we want to block it in order for the 3-way
	 * negotiation to have matching parameters */
	err = phylink_connect_phy(sfp->phylink, phy);
	if (err) {
		phy_device_remove(phy);
		phy_device_free(phy);
		dev_err(sfp->dev, "phylink_connect_phy failed: %d\n", err);
		return;
	}

	/* We found a PHY.  Switch the link to PHY mode, so we use phylib
	 * to feed the negotiation results to the MAC.  This avoids the
	 * question over which Serdes mode the PHY is operating.
	 * XXX: It is not wise to set-up SGMII autoneg here because some PHY (like finisar FCLF-8521-3) transfer base-X aneg
	 */
	sfp->mod_phy = phy;
	sfp_scan_phy_fixups(sfp, false);	/* scan for vendor and and pn and register phy_fixup before phy_start */
	phylink_set_link_an_mode(sfp->phylink, MLO_AN_PHY);
	phy_start(phy);
}

static void sfp_sm_link_up(struct sfp *sfp)
{
	if (sfp->phylink)
		phylink_enable(sfp->phylink);
	else
		netdev_dbg(sfp->ndev, "NO PHYLINK!!!!!");

	sfp_sm_next(sfp, SFP_S_LINK_UP, 0);
}

static void sfp_sm_link_down(struct sfp *sfp)
{
	if (sfp->phylink)
		phylink_disable(sfp->phylink);
}

static void sfp_sm_link_check_los(struct sfp *sfp)
{
	unsigned int los = sfp->state & SFP_F_LOS;

	/* FIXME: what if neither SFP_OPTIONS_LOS_INVERTED nor
	 * SFP_OPTIONS_LOS_NORMAL are set?  For now, we assume
	 * the same as SFP_OPTIONS_LOS_NORMAL set.
	 */
	if (sfp->id.ext.options & SFP_OPTIONS_LOS_INVERTED)
		los ^= SFP_F_LOS;

	if (los)
		sfp_sm_next(sfp, SFP_S_WAIT_LOS, 0);
	else
		sfp_sm_link_up(sfp);
}

static void sfp_sm_fault(struct sfp *sfp, bool warn)
{
	if (sfp->sm_retries && !--sfp->sm_retries) {
		dev_err(sfp->dev, "module persistently indicates fault, disabling\n");
		sfp_sm_next(sfp, SFP_S_TX_DISABLE, 0);
	} else {
		if (warn)
			dev_err(sfp->dev, "module transmit fault indicated\n");

		sfp_sm_next(sfp, SFP_S_TX_FAULT, T_FAULT_RECOVER);
	}
}

static void sfp_sm_mod_init(struct sfp *sfp)
{
	sfp_module_tx_enable(sfp);

	/* Wait t_init before indicating that the link is up, provided the
	 * current state indicates no TX_FAULT.  If TX_FAULT clears before
	 * this time, that's fine too.
	 */
	sfp_sm_next(sfp, SFP_S_INIT, T_INIT_JIFFIES);
	sfp->sm_retries = 5;

	if (sfp->phylink && sfp->id.base.e1000_base_t) {
		netdev_dbg(sfp->ndev, "Will probe phy");
		sfp_sm_probe_phy(sfp);
	} else {
		netdev_dbg(sfp->ndev, "Not probing phy. %d", !!(sfp->phylink));
	}
	
}

static int sfp_sm_mod_probe(struct sfp *sfp)
{
	/* SFP module inserted - read I2C data */
	struct sfp_eeprom_id id;
	char vendor[17];
	char part[17];
	char sn[17];
	char date[9];
	char options[80];
	char rev[5];
	u8 check;
	int err;

	err = sfp_read(sfp, false, 0, &id, sizeof(id));
	if (err < 0) {
		dev_dbg(sfp->dev, "failed to read EEPROM: %d\n", err);
		return -EAGAIN;
	}

	/* Validate the checksum over the base structure */
	check = sfp_check(&id.base, sizeof(id.base) - 1);
	if (check != id.base.cc_base) {
		dev_err(sfp->dev,
			"EEPROM base structure checksum failure: 0x%02x (0x%x)\n",
			check, id.base.cc_base);
		return -EINVAL;
	}

	check = sfp_check(&id.ext, sizeof(id.ext) - 1);
	if (check != id.ext.cc_ext) {
		dev_err(sfp->dev,
			"EEPROM extended structure checksum failure: 0x%02x (0x%x)\n",
			check, id.ext.cc_ext);
		memset(&id.ext, 0, sizeof(id.ext));
	}

	sfp->id = id;

	memcpy(vendor, sfp->id.base.vendor_name, 16);
	vendor[16] = '\0';
	memcpy(part, sfp->id.base.vendor_pn, 16);
	part[16] = '\0';
	memcpy(rev, sfp->id.base.vendor_rev, 4);
	rev[4] = '\0';
	memcpy(sn, sfp->id.ext.vendor_sn, 16);
	sn[16] = '\0';
//	memcpy(date, sfp->id.ext.datecode, 8);
	date[0] = sfp->id.ext.datecode[4];
	date[1] = sfp->id.ext.datecode[5];
	date[2] = '-';
	date[3] = sfp->id.ext.datecode[2];
	date[4] = sfp->id.ext.datecode[3];
	date[5] = '-';
	date[6] = sfp->id.ext.datecode[0];
	date[7] = sfp->id.ext.datecode[1];
	date[8] = '\0';

	dev_info(sfp->dev, "module %s %s rev %s sn %s dc %s\n", vendor, part, rev, sn, date);
	dev_info(sfp->dev, "  %s connector, encoding %s, nominal bitrate %u.%uGbps +%u%% -%u%%\n",
		 sfp_connector(sfp->id.base.connector),
		 sfp_encoding(sfp->id.base.encoding),
		 sfp->id.base.br_nominal / 10,
		 sfp->id.base.br_nominal % 10,
		 sfp->id.ext.br_max, sfp->id.ext.br_min);
	dev_info(sfp->dev, "  1000BaseSX%c 1000BaseLX%c 1000BaseCX%c 1000BaseT%c 100BaseTLX%c 1000BaseFX%c BaseBX10%c BasePX%c\n",
		 sfp->id.base.e1000_base_sx ? '+' : '-',
		 sfp->id.base.e1000_base_lx ? '+' : '-',
		 sfp->id.base.e1000_base_cx ? '+' : '-',
		 sfp->id.base.e1000_base_t ? '+' : '-',
		 sfp->id.base.e100_base_lx ? '+' : '-',
		 sfp->id.base.e100_base_fx ? '+' : '-',
		 sfp->id.base.e_base_bx10 ? '+' : '-',
		 sfp->id.base.e_base_px ? '+' : '-');

	if (!sfp->id.base.sfp_ct_passive && !sfp->id.base.sfp_ct_active &&
		!sfp->id.base.e1000_base_t) {
		char len_9um[16], len_om[16];

		dev_info(sfp->dev, "  Wavelength %unm, fiber lengths:\n",
			 be16_to_cpup(&sfp->id.base.optical_wavelength));

		if (sfp->id.base.link_len[0] == 255)
			strcpy(len_9um, ">254km");
		else if (sfp->id.base.link_len[1] && sfp->id.base.link_len[1] != 255)
			sprintf(len_9um, "%um",
				sfp->id.base.link_len[1] * 100);
		else if (sfp->id.base.link_len[0])
			sprintf(len_9um, "%ukm", sfp->id.base.link_len[0]);
		else if (sfp->id.base.link_len[1] == 255)
			strcpy(len_9um, ">25.4km");
		else
			strcpy(len_9um, "unsupported");

		dev_info(sfp->dev, "	9µm SM	: %s\n", len_9um);
		dev_info(sfp->dev, " 62.5µm MM OM1: %s\n",
			 sfp_link_len(len_om, sizeof(len_om),
					  sfp->id.base.link_len[3], 10));
		dev_info(sfp->dev, "   50µm MM OM2: %s\n",
			 sfp_link_len(len_om, sizeof(len_om),
					  sfp->id.base.link_len[2], 10));
		dev_info(sfp->dev, "   50µm MM OM3: %s\n",
			 sfp_link_len(len_om, sizeof(len_om),
					  sfp->id.base.link_len[5], 10));
		dev_info(sfp->dev, "   50µm MM OM4: %s\n",
			 sfp_link_len(len_om, sizeof(len_om),
					  sfp->id.base.link_len[4], 10));
	} else {
		char len[16];
		dev_info(sfp->dev, "  Copper length: %s\n",
			 sfp_link_len(len, sizeof(len),
					  sfp->id.base.link_len[4], 1));
	}

	dev_info(sfp->dev, "  Options: %s\n",
		 sfp_bitfield(options, sizeof(options), sfp_options,
				  be16_to_cpu(sfp->id.ext.options)));
	dev_info(sfp->dev, "  Diagnostics: %s\n",
		 sfp_bitfield(options, sizeof(options), diagmon,
				  sfp->id.ext.diagmon));
	/* We only support SFP modules, not the legacy GBIC modules. */
	if (sfp->id.base.phys_id != SFP_PHYS_ID_SFP ||
	    sfp->id.base.phys_ext_id != SFP_PHYS_EXT_ID_SFP) {
		dev_err(sfp->dev, "module is not SFP - phys id 0x%02x 0x%02x\n",
			sfp->id.base.phys_id, sfp->id.base.phys_ext_id);
		return -EINVAL;
	}

	/*
	 * What isn't clear from the SFP documentation is whether this
	 * specifies the encoding expected on the TD/RD lines, or whether
	 * the TD/RD lines are always 8b10b encoded, but the transceiver
	 * converts.  Eg, think of a copper SFP supporting 1G/100M/10M
	 * ethernet: this requires 8b10b encoding for 1G, 4b5b for 100M,
	 * and manchester for 10M.
	 */
	/* 1Gbit ethernet requires 8b10b encoding */
	if (sfp->id.base.encoding != SFP_ENCODING_8B10B) 
		dev_err(sfp->dev, "module does not support 8B10B encoding (0x%02x) (baseT %d). Will try 100base-FX!\n", sfp->id.base.encoding, !!(sfp->id.base.e1000_base_t));
		
	if (sfp->phylink) {
		u32 support;
		u8 port;

		if (sfp->id.base.e1000_base_t) {
			support = SUPPORTED_TP;
			port = PORT_TP;
		} else {
			support = SUPPORTED_FIBRE;
			port = PORT_FIBRE;
		}
		phylink_set_link_port(sfp->phylink, support, port);

		/* Setting the serdes link mode is guesswork: there's no
		 * field in the EEPROM which indicates what mode should
		 * be used.
		 *
		 * If it's a fiber module, it probably does not have a
		 * PHY, so switch to 802.3z negotiation mode.
		 *
		 * If it's a copper module, try SGMII mode.  If that
		 * doesn't work, we should try switching to 802.3z mode.
		 */
		if (sfp->id.base.encoding != SFP_ENCODING_8B10B)
			phylink_set_link_an_mode(sfp->phylink, MLO_AN_100FX);
		else if (sfp->id.base.e1000_base_t)
			phylink_set_link_an_mode(sfp->phylink, MLO_AN_SGMII);
		else
			phylink_set_link_an_mode(sfp->phylink, MLO_AN_8023Z);
		#if 0
		if (!sfp->id.base.e1000_base_t) {
//n			sfp_change_phy_basex(sfp, true);
			phylink_set_link_an_mode(sfp->phylink, MLO_AN_8023Z);
		} else {
//			sfp_change_phy_basex(sfp, false);
			phylink_set_link_an_mode(sfp->phylink, MLO_AN_SGMII);
		}
		#endif
	} 

	return 0;
}

static void sfp_sm_mod_remove(struct sfp *sfp)
{
	if (sfp->mod_phy)
		sfp_sm_phy_detach(sfp);

	sfp_module_tx_disable(sfp);

	memset(&sfp->id, 0, sizeof(sfp->id));

	dev_info(sfp->dev, "module removed\n");
}

static void sfp_sm_event(struct sfp *sfp, unsigned int event)
{
	mutex_lock(&sfp->sm_mutex);

//	dev_dbg(sfp->dev, "SM: enter %u:%u:%u event %u\n",
//		sfp->sm_mod_state, sfp->sm_dev_state, sfp->sm_state, event);
	netdev_dbg(sfp->ndev, "SM: enter module(%s[%u]):device(%s[%u]):sm(%s[%u]) event %s[%u]", mod_state_to_str(sfp->sm_mod_state), sfp->sm_mod_state, 
									dev_state_to_str(sfp->sm_dev_state), sfp->sm_dev_state, 
										main_sm_to_str(sfp->sm_state), sfp->sm_state, event_to_str(event), event);
	/* This state machine tracks the insert/remove state of
	 * the module, and handles probing the on-board EEPROM.
	 */
	switch (sfp->sm_mod_state) {
	default:
		if (event == SFP_E_INSERT || event == SFP_E_REMOVE) {
			sfp_module_tx_disable(sfp);
			sfp_sm_ins_next(sfp, SFP_MOD_PROBE, T_PROBE_INIT);
		}
		break;

	case SFP_MOD_PROBE:
		if (event == SFP_E_REMOVE) {
			sfp_sm_ins_next(sfp, SFP_MOD_EMPTY, 0);
		} else if (event == SFP_E_TIMEOUT) {
			int err = sfp_sm_mod_probe(sfp);
			if (err == 0)
				sfp_sm_ins_next(sfp, SFP_MOD_PRESENT, 0);
			else if (err == -EAGAIN)
				sfp_sm_set_timer(sfp, T_PROBE_RETRY);
			else
				sfp_sm_ins_next(sfp, SFP_MOD_ERROR, 0);
		}
		break;

	case SFP_MOD_PRESENT:
	case SFP_MOD_ERROR:
		if (event == SFP_E_REMOVE) {
			sfp_sm_mod_remove(sfp);
			sfp_sm_ins_next(sfp, SFP_MOD_EMPTY, 0);
		}
		break;
	}

//	printk(KERN_ERR "%s-%d netdev state sm: dev state %d, event %d\n", __func__, __LINE__, sfp->sm_dev_state, event);
	/* This state machine tracks the netdev up/down state */
	switch (sfp->sm_dev_state) {
	default:
		if (event == SFP_E_DEV_UP) 
			sfp->sm_dev_state = SFP_DEV_UP;
		break;

	case SFP_DEV_UP:
		if (event == SFP_E_DEV_DOWN) {
			/* If the module has a PHY, avoid raising TX disable
			 * as this resets the PHY. Otherwise, raise it to
			 * turn the laser off.
			 */
			if (!sfp->mod_phy)
				sfp_module_tx_disable(sfp);
			sfp->sm_dev_state = SFP_DEV_DOWN;
		}
		break;
	}

	/* Some events are global */
	if (sfp->sm_state != SFP_S_DOWN &&
	    (sfp->sm_mod_state != SFP_MOD_PRESENT ||
	     sfp->sm_dev_state != SFP_DEV_UP)) {
		if (sfp->sm_state == SFP_S_LINK_UP &&
		    sfp->sm_dev_state == SFP_DEV_UP) 
			sfp_sm_link_down(sfp);
		if (sfp->mod_phy) 
			sfp_sm_phy_detach(sfp);
		sfp_sm_next(sfp, SFP_S_DOWN, 0);
		mutex_unlock(&sfp->sm_mutex);
		return;
	}

	/* The main state machine */
	switch (sfp->sm_state) {
	case SFP_S_DOWN:
		if (sfp->sm_mod_state == SFP_MOD_PRESENT &&
		    sfp->sm_dev_state == SFP_DEV_UP)
			sfp_sm_mod_init(sfp);
		break;

	case SFP_S_INIT:
		if (event == SFP_E_TIMEOUT && sfp->state & SFP_F_TX_FAULT)
			sfp_sm_fault(sfp, true);
		else if (event == SFP_E_TIMEOUT || event == SFP_E_TX_CLEAR)
			sfp_sm_link_check_los(sfp);
		break;

	case SFP_S_WAIT_LOS:
		if (event == SFP_E_TX_FAULT)
			sfp_sm_fault(sfp, true);
		else if (event ==
			 (sfp->id.ext.options & SFP_OPTIONS_LOS_INVERTED ?
			  SFP_E_LOS_HIGH : SFP_E_LOS_LOW))
			sfp_sm_link_up(sfp);
		break;

	case SFP_S_LINK_UP:
		if (event == SFP_E_TX_FAULT) {
			sfp_sm_link_down(sfp);
			sfp_sm_fault(sfp, true);
		} else if (event ==
			   (sfp->id.ext.options & SFP_OPTIONS_LOS_INVERTED ?
			    SFP_E_LOS_LOW : SFP_E_LOS_HIGH)) {
			dev_info(sfp->dev, "LOS detected. disabling sfp\n");
			sfp_sm_link_down(sfp);
			sfp_sm_next(sfp, SFP_S_WAIT_LOS, 0);
		}
		break;

	case SFP_S_TX_FAULT:
		if (event == SFP_E_TIMEOUT) {
			sfp_module_tx_fault_reset(sfp);
			sfp_sm_next(sfp, SFP_S_REINIT, T_INIT_JIFFIES);
		}
		break;

	case SFP_S_REINIT:
		if (event == SFP_E_TIMEOUT && sfp->state & SFP_F_TX_FAULT) {
			sfp_sm_fault(sfp, false);
		} else if (event == SFP_E_TIMEOUT || event == SFP_E_TX_CLEAR) {
			dev_info(sfp->dev, "module transmit fault recovered\n");
			sfp_sm_link_check_los(sfp);
		}
		break;

	case SFP_S_TX_DISABLE:
		break;
	}

//	dev_err(sfp->dev, "SM: exit %u:%u:%u\n",
//		sfp->sm_mod_state, sfp->sm_dev_state, sfp->sm_state);

	netdev_dbg(sfp->ndev, "SM: exit %u:%u:%u", sfp->sm_mod_state, sfp->sm_dev_state, sfp->sm_state);
	mutex_unlock(&sfp->sm_mutex);
}

/* sysfs routines */
#if 0
static inline ssize_t _print_eeprom(struct sfp_eeprom_id *id, char *buf)
{
	ssize_t ret = strlen(buf);
	struct sfp_eeprom_base *base = &id->base;
	struct sfp_eeprom_ext *ext = &id->ext;
	char date[9], options[80];


	date[0] = ext->datecode[4];
	date[1] = ext->datecode[5];
	date[2] = '-';
	date[3] = ext->datecode[2];
	date[4] = ext->datecode[3];
	date[5] = '-';
	date[6] = ext->datecode[0];
	date[7] = ext->datecode[1];
	date[8] = '\0';

	ret += sprintf(&buf[ret], "UniqueID: %s, S/N: %s\n", base->vendor_oui, ext->vendor_sn);
	ret += sprintf(&buf[ret], "Vendor: %s, PN: %s, REV: %s, Date: %s\n", base->vendor_name, base->vendor_pn, base->vendor_rev, date);
	ret += sprintf(&buf[ret], "%s connector, encoding %s, nominal bitrate %u.%uGbps +%u%% -%u%%\n",
		 sfp_connector(base->connector),
		 sfp_encoding(base->encoding),
		 base->br_nominal / 10,
		 base->br_nominal % 10,
		 ext->br_max, ext->br_min);
	ret += sprintf(&buf[ret], "1000BaseSX%c 1000BaseLX%c 1000BaseCX%c 1000BaseT%c 100BaseTLX%c 1000BaseFX%c BaseBX10%c BasePX%c\n",
		 base->e1000_base_sx ? '+' : '-',
		 base->e1000_base_lx ? '+' : '-',
		 base->e1000_base_cx ? '+' : '-',
		 base->e1000_base_t ? '+' : '-',
		 base->e100_base_lx ? '+' : '-',
		 base->e100_base_fx ? '+' : '-',
		 base->e_base_bx10 ? '+' : '-',
		 base->e_base_px ? '+' : '-');

	if (!base->sfp_ct_passive && !base->sfp_ct_active &&
		!base->e1000_base_t) {
		char len_9um[16], len_om[16];

		ret += sprintf(&buf[ret], "  Wavelength %unm, fiber lengths:\n",
			 be16_to_cpup(&base->optical_wavelength));

		if (base->link_len[0] == 255)
			strcpy(len_9um, ">254km");
		else if (base->link_len[1] && base->link_len[1] != 255)
			sprintf(len_9um, "%um",
				base->link_len[1] * 100);
		else if (base->link_len[0])
			sprintf(len_9um, "%ukm", base->link_len[0]);
		else if (base->link_len[1] == 255)
			strcpy(len_9um, ">25.4km");
		else
			strcpy(len_9um, "unsupported");

		ret += sprintf(&buf[ret], "	9µm SM	: %s\n", len_9um);
		ret += sprintf(&buf[ret], " 62.5µm MM OM1: %s\n",
			 sfp_link_len(len_om, sizeof(len_om),
					  base->link_len[3], 10));
		ret += sprintf(&buf[ret], "   50µm MM OM2: %s\n",
			 sfp_link_len(len_om, sizeof(len_om),
					  base->link_len[2], 10));
		ret += sprintf(&buf[ret], "   50µm MM OM3: %s\n",
			 sfp_link_len(len_om, sizeof(len_om),
					  base->link_len[5], 10));
		ret += sprintf(&buf[ret], "   50µm MM OM4: %s\n",
			 sfp_link_len(len_om, sizeof(len_om),
					  base->link_len[4], 10));
	} else {
		char len[16];
		ret += sprintf(&buf[ret], "  Copper length: %s\n",
			 sfp_link_len(len, sizeof(len),
					  base->link_len[4], 1));
	}

	ret += sprintf(&buf[ret], "  Options: %s\n",
		 sfp_bitfield(options, sizeof(options), sfp_options,
				  be16_to_cpu(ext->options)));
	ret += sprintf(&buf[ret], "  Diagnostics: %s\n",
		 sfp_bitfield(options, sizeof(options), diagmon,
				  ext->diagmon));
	/* We only support SFP modules, not the legacy GBIC modules. */
	if (base->phys_id != SFP_PHYS_ID_SFP ||
	    base->phys_ext_id != SFP_PHYS_EXT_ID_SFP) {
		ret += sprintf(&buf[ret], "module is not SFP - phys id 0x%02x 0x%02x\n",
			base->phys_id, base->phys_ext_id);
		return -EINVAL;
	}

	if (base->encoding != SFP_ENCODING_8B10B) 
		ret += sprintf(&buf[ret], "module does not support 8B10B encoding\n");
	 else
	 	ret += sprintf(&buf[ret], "module supports 8B10B encoding\n");

	return ret;
}
#endif 

static ssize_t dump_sfp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sfp_plat_data *pdata = pdev->dev.platform_data;
	struct sfp *sfp = platform_get_drvdata(pdev);
	int state, i;
	const char *state_str = NULL;
	ssize_t ret = 0;
	
	ret += sprintf(buf, "SFP eth%d [i2c-%d] info:\n", sfp->port, pdata->i2c_bus);
	ret += sprintf(&buf[ret], "STATE MACHINE: Module:%s, NetDevice State: %s, Main state: %s\n", 
				mod_state_to_str(sfp->sm_mod_state), dev_state_to_str(sfp->sm_dev_state), main_sm_to_str(sfp->sm_state));
	state = sfp_get_state(sfp);
	for (i = 0; i < GPIO_MAX; i++) 
		ret += sprintf(&buf[ret], "%s[%u] : %u\n", gpio_of_names[i], (sfp->gpio[i] ? sfp->gpio[i] : 0), !!(state & BIT(i)));
	if (sfp->mod_phy) {
		state_str = get_phy_state_str(sfp->mod_phy, &state);
		if (state_str != NULL) 
			ret += sprintf(&buf[ret], "SFP PHY state[%d] %s\n", state, state_str);
		else 
			ret += sprintf(&buf[ret], "SFP PHY state Unknwon %d\n", state);
	} else {
		ret += sprintf(&buf[ret], "No SFP PHY\n");
	}
	ret += sprintf(&buf[ret], "\nEEPROM:\n");
	return ret;
//	return (ret + _print_eeprom(id, buf));
}
static DEVICE_ATTR(dump_sfp_info, 0444, dump_sfp, NULL);

#if 0
static ssize_t sfp_switch_to_sgmii(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sfp *sfp = platform_get_drvdata(pdev);
	int val = (int)simple_strtol(buf, NULL, 10);
	int ret = 0;
		
	if (sfp->phylink) {
		if (val != 0) {
			phylink_set_link_port(sfp->phylink, SUPPORTED_TP, PORT_TP);
			ret = phylink_set_link_an_mode(sfp->phylink, MLO_AN_SGMII);
		} else {
			phylink_set_link_port(sfp->phylink, SUPPORTED_FIBRE, PORT_FIBRE);
			ret = phylink_set_link_an_mode(sfp->phylink, MLO_AN_8023Z);
		}
		if (ret != 0) {
			dev_err(&pdev->dev, "%s error %d in phylink_set_link_an_mode()\n", __func__, ret);
			return -EAGAIN;
		}
		return count;
	}
	return -EAGAIN;
}
static DEVICE_ATTR(switch_to_sgmii, 0644, NULL, sfp_switch_to_sgmii);

#endif
#if 0
static int sfp_phy_module_info(struct phy_device *phy, struct ethtool_modinfo *modinfo)
{
	struct sfp *sfp = phy->priv;

	/* locking... and check module is present */

	if (sfp->id.ext.sff8472_compliance) {
		modinfo->type = ETH_MODULE_SFF_8472;
		modinfo->eeprom_len = ETH_MODULE_SFF_8472_LEN;
	} else {
		modinfo->type = ETH_MODULE_SFF_8079;
		modinfo->eeprom_len = ETH_MODULE_SFF_8079_LEN;
	}
	return 0;
}

static int sfp_phy_module_eeprom(struct phy_device *phy,
	struct ethtool_eeprom *ee, u8 *data)
{
	struct sfp *sfp = phy->priv;
	unsigned int first, last, len;
	int ret;

	if (ee->len == 0)
		return -EINVAL;

	first = ee->offset;
	last = ee->offset + ee->len;
	if (first < ETH_MODULE_SFF_8079_LEN) {
		len = last;
		if (len > ETH_MODULE_SFF_8079_LEN)
			len = ETH_MODULE_SFF_8079_LEN;
		len -= first;

		ret = sfp->read(sfp, false, first, data, len);
		if (ret < 0)
			return ret;

		first += len;
		data += len;
	}
	if (first >= ETH_MODULE_SFF_8079_LEN && last > first) {
		len = last - first;

		ret = sfp->read(sfp, true, first, data, len);
		if (ret < 0)
			return ret;
	}
	return 0;
}
#endif

static void sfp_timeout(struct work_struct *work)
{
	struct sfp *sfp = container_of(work, struct sfp, timeout.work);

	sfp_sm_event(sfp, SFP_E_TIMEOUT);
}

static void sfp_check_state(struct sfp *sfp)
{
	unsigned int state, i, changed;

	state = sfp_get_state(sfp);
	changed = state ^ sfp->state;
	changed &= SFP_F_PRESENT | SFP_F_LOS | SFP_F_TX_FAULT;

	for (i = 0; i < GPIO_MAX; i++)
		if (changed & BIT(i))
			dev_dbg(sfp->dev, "%s %u -> %u\n", gpio_of_names[i],
				!!(sfp->state & BIT(i)), !!(state & BIT(i)));

	state |= sfp->state & (SFP_F_TX_DISABLE | SFP_F_RATE_SELECT);
	sfp->state = state;

	if (changed & SFP_F_PRESENT) {
		sfp_sm_event(sfp, state & SFP_F_PRESENT ?
				SFP_E_INSERT : SFP_E_REMOVE); 
	}

	if (changed & SFP_F_TX_FAULT)
		sfp_sm_event(sfp, state & SFP_F_TX_FAULT ?
				SFP_E_TX_FAULT : SFP_E_TX_CLEAR);

	if (changed & SFP_F_LOS)
		sfp_sm_event(sfp, state & SFP_F_LOS ?
				SFP_E_LOS_HIGH : SFP_E_LOS_LOW);
}

static irqreturn_t sfp_irq(int irq, void *data)
{
	struct sfp *sfp = data;

	sfp_check_state(sfp);

	return IRQ_HANDLED;
}

static void sfp_poll(struct work_struct *work)
{
	struct sfp *sfp = container_of(work, struct sfp, poll.work);

	sfp_check_state(sfp);
	#ifdef NOW_SUPPORTED
	mod_delayed_work(system_wq, &sfp->poll, poll_jiffies);
	#else
//	queue_delayed_work(system_wq, &sfp->poll, (unsigned long)poll_jiffies);
	schedule_delayed_work(&sfp->poll,  (unsigned long)poll_jiffies);
	#endif
}

static inline const char *action_to_str(unsigned long act)
{
	switch (act) {
	case NETDEV_UP:
		return "UP";
	case NETDEV_GOING_DOWN:
		return "GOING DOWN";
	case NETDEV_UNREGISTER:
		return "UNREGISTER";
	default:
		return "UNHANDLED";
	}
}
	

static int sfp_netdev_notify(struct notifier_block *nb, unsigned long act, void *data)
{
	struct sfp *sfp = container_of(nb, struct sfp, netdev_nb);
//	struct netdev_notifier_info *info = data;
	struct net_device *ndev = data;

	netdev_dbg(sfp->ndev, "Called. action %s (%lu)", action_to_str(act), (act));

	if (!sfp->ndev || ndev != sfp->ndev)
		return NOTIFY_DONE;
	
	switch (act) {
	case NETDEV_UP:
		sfp_sm_event(sfp, SFP_E_DEV_UP);
		break;

	case NETDEV_GOING_DOWN:
		sfp_sm_event(sfp, SFP_E_DEV_DOWN);
		break;

	case NETDEV_UNREGISTER:
		if (sfp->mod_phy && sfp->phylink)
			phylink_disconnect_phy(sfp->phylink);
		sfp->phylink = NULL;
		dev_put(sfp->ndev);
		sfp->ndev = NULL;
		break;
	}
	return NOTIFY_OK;
}

static struct sfp *sfp_alloc(struct device *dev)
{
	struct sfp *sfp;

	sfp = kzalloc(sizeof(*sfp), GFP_KERNEL);
	if (!sfp)
		return ERR_PTR(-ENOMEM);

	sfp->dev = dev;

	mutex_init(&sfp->sm_mutex);
	INIT_DELAYED_WORK(&sfp->poll, sfp_poll);
	INIT_DELAYED_WORK(&sfp->timeout, sfp_timeout);

	sfp->netdev_nb.notifier_call = sfp_netdev_notify;

	return sfp;
}

static void sfp_destroy(struct sfp *sfp)
{
	cancel_delayed_work_sync(&sfp->poll);
	cancel_delayed_work_sync(&sfp->timeout);
	if (sfp->i2c_mii) {
		mdiobus_unregister(sfp->i2c_mii);
		mdiobus_free(sfp->i2c_mii);
	}
	if (sfp->i2c)
		i2c_put_adapter(sfp->i2c);
	#ifdef NOW_SUPPORTED
	of_node_put(sfp->dev->of_node);
	#else
	kobject_put(&sfp->dev->kobj);
	#endif
	kfree(sfp);
}

static void sfp_cleanup(void *data)
{
	struct sfp *sfp = data;

	sfp_destroy(sfp);
}

static int sfp_probe(struct platform_device *pdev)
{
	struct sfp *sfp;
	char if_name[5] = "ethX";
	bool poll = false;
	int irq, err, i;

	sfp = sfp_alloc(&pdev->dev);
	if (IS_ERR(sfp)) {
		dev_err(&pdev->dev, "Couldn't alloc");
		return PTR_ERR(sfp);
	}

	platform_set_drvdata(pdev, sfp);

	#ifdef NOW_SUPPORTED
	err = devm_add_action(sfp->dev, sfp_cleanup, sfp);
	if (err < 0)
		return err;
	#endif

	if (pdev->dev.platform_data) {
		struct sfp_plat_data 	*pdata = pdev->dev.platform_data;
		struct i2c_adapter 	*adap;

		if (pdata->i2c_bus < 1) {
			dev_err(&pdev->dev, "%s Must provide i2c bus, %d invalid\n", __func__, pdata->i2c_bus);
			return -EINVAL;
		}
	
		adap = i2c_get_adapter(pdata->i2c_bus);
		if (!adap) { 
			dev_err(&pdev->dev, "error getting adap %d\n", pdata->i2c_bus);
			return -EIO;
		}
		err = sfp_i2c_configure(sfp, adap);
		if (err < 0) {	
			i2c_put_adapter(adap);
			dev_err(&pdev->dev, "Error %d configuring i2c adap\n", err);
			return err;
		} 

		/* set up control gpios */
		for (i = 0; i < GPIO_MAX; i++) {
			int *val = (int*)&(pdata->ctrl.gpio_mod0);
			val += i;
			if (*val) 
				sfp->gpio[i] = *val;
			else
				sfp->gpio[i] = 0;

			if (sfp->gpio[i]) {
				err = gpio_request(sfp->gpio[i], gpio_of_names[i]);
				if (err != 0) {
					dev_err(&pdev->dev, "Error %d requesting gpio %d (%s)\n", err, sfp->gpio[i], gpio_of_names[i]);
					return err;
				}
				gpio_direction_input(sfp->gpio[i]);
				gpio_export(sfp->gpio[i], true);
			}
		}

		sfp->get_state = sfp_gpio_get_state;
		sfp->set_state = sfp_gpio_set_state;

		sprintf(if_name, "eth%d", pdata->port);
//		sfp->ndev = dev_get_by_index(&init_net, pdata->port);
		sfp->ndev = dev_get_by_name(&init_net, if_name);
		if (!sfp->ndev) {
			dev_err(&pdev->dev, "ndev ethernet device not found\n");
			return -EAGAIN;
		}
		sfp->port = pdata->port;

		dev_hold(sfp->ndev);
		put_device(&sfp->ndev->dev);

		sfp->phylink = phylink_lookup_by_netdev(sfp->ndev);
		if (!sfp->phylink) {
			dev_err(&pdev->dev, "phylink ethernet device not found\n");
			return -EAGAIN;
		}

		phylink_disable(sfp->phylink);
	} else {
		pr_err("No platform data and no DT support. Aborting.\n");
		return -EINVAL;
	}

	sfp->state = sfp_get_state(sfp);
	if (sfp->gpio[GPIO_TX_DISABLE] &&
	    gpio_get_value_cansleep(sfp->gpio[GPIO_TX_DISABLE]))
		sfp->state |= SFP_F_TX_DISABLE;
	if (sfp->gpio[GPIO_RATE_SELECT] &&
	    gpio_get_value_cansleep(sfp->gpio[GPIO_RATE_SELECT]))
		sfp->state |= SFP_F_RATE_SELECT;
	sfp_set_state(sfp, sfp->state);
	sfp_module_tx_disable(sfp);
	if (sfp->state & SFP_F_PRESENT) {
		sfp_sm_event(sfp, SFP_E_INSERT);	/* NOTE: Boris-Ben Shapiro (2016-07-31): opposite logic */
	}

	for (i = 0; i < GPIO_MAX; i++) {
		if (gpio_flags[i] != GPIOD_IN || !sfp->gpio[i])
			continue;

		irq = gpio_to_irq(sfp->gpio[i]);
		if (!irq) {
			poll = true;
			continue;
		}
		dev_info(&pdev->dev, "%d: irq %d for gpio %d (%s)\n", i, irq, sfp->gpio[i], gpio_of_names[i]);

		err = devm_request_threaded_irq(sfp->dev, irq, NULL, sfp_irq,
						IRQF_ONESHOT |
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING,
						dev_name(sfp->dev), sfp);
		if (err) {
			dev_err(&pdev->dev, "%s error %d requesting gpio %d for %s. poll ok!!\n", __func__, err, sfp->gpio[i], gpio_of_names[i]);
			poll = true;
		}
	}

	if (poll) {
		#ifdef NOW_SUPPORTED
		mod_delayed_work(system_wq, &sfp->poll, poll_jiffies);
		#else
//		queue_delayed_work(system_wq, &sfp->poll, (unsigned long)poll_jiffies);
		schedule_delayed_work(&sfp->poll, (unsigned long)poll_jiffies);
		#endif
	}

	err = register_netdevice_notifier(&sfp->netdev_nb);
	dev_dbg(&pdev->dev, "%s registering netdevice_notifier ret %d\n", __func__, err);

	err = device_create_file(sfp->dev, &dev_attr_dump_sfp_info);
	if (err) {
		dev_err(&pdev->dev, "%s error %d registering sysfs file\n", __func__, err);
		return err;
	}
	#if 0
	err = device_create_file(sfp->dev, &dev_attr_switch_to_sgmii);
	if (err) {
		dev_err(&pdev->dev, "%s error %d registering sysfs file switch to sgmii\n", __func__, err);
		return err;
	}
	#endif


	return 0;
}

static int sfp_remove(struct platform_device *pdev)
{
	struct sfp *sfp = platform_get_drvdata(pdev);

	#if 0
	device_remove_file(sfp->dev, &dev_attr_switch_to_sgmii);
	#endif
	device_remove_file(sfp->dev, &dev_attr_dump_sfp_info);
	unregister_netdevice_notifier(&sfp->netdev_nb);
	if (sfp->ndev)
		dev_put(sfp->ndev);

	sfp_cleanup(sfp);

	return 0;
}

#ifdef NOW_SUPPORTED
static const struct of_device_id sfp_of_match[] = {
	{ .compatible = "sff,sfp", },
	{ },
};
MODULE_DEVICE_TABLE(of, sfp_of_match);
#endif

static struct platform_driver sfp_driver = {
	.probe = sfp_probe,
	.remove = sfp_remove,
	.driver = {
		.name = "sfp",
		#ifdef NOW_SUPPORTED
		.of_match_table = sfp_of_match,
		#else
		.owner = THIS_MODULE,
		#endif
	},
};

static int sfp_init(void)
{
	poll_jiffies = msecs_to_jiffies(100);

	return platform_driver_register(&sfp_driver);
}
late_initcall(sfp_init);

static void sfp_exit(void)
{
	platform_driver_unregister(&sfp_driver);
}
module_exit(sfp_exit);

/* NOTE: Boris-Ben Shapiro (2016-08-30) Fixup functions */
/* See https://www.finisar.com/sites/default/files/resources/an-2036_fcxx-852x-3_faqrevd1corrected_updated.pdf */
static int finiar_fclf_phy_fixup(struct phy_device *phy)
{
	u8 addrs[5] = {0x1B, 0x09, 0x00, 0x04, 0x00};
	u16 vals[5] = {0x9084, 0x0F00, 0x8140, 0x0DE1, 0x9140};
	int ret = 0, i;

	pr_info("Detected sfp module finisar fclf 8521-3. Switching to SGMII\n");
	if (!phy) {
		dev_err(&phy->dev, "%s No phy probed yet!!!\n", __func__);
		return -ENODEV;
	}
	/* Check if fixup applied */
	for (i = 0; i < 5; i++) {
		ret = phy_write(phy, addrs[i], vals[i]);	/* Enable SGMII */
		if (ret) {
			dev_err(&phy->dev, "%s Error %d writing 0x%04x to phy reg 0x%02x\n", __func__, ret, vals[i], addrs[i]);
			return ret;
		}
		#ifdef DEBUG
		ret = phy_read(phy, addrs[i]);
		dev_err(&phy->dev, "%s rereadphy 0x%02x: reg 0x%02x val 0x%04x\n", __func__, phy->phy_id, addrs[i], (u16)ret);
		#endif
	}
	pr_info("Succsfully applied fixup for FINISAR FCLF-8521\n");
	return 0;
}

MODULE_ALIAS("platform:sfp");
MODULE_AUTHOR("Russell King");
MODULE_LICENSE("GPL v2");
