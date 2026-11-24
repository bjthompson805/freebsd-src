/*	$OpenBSD: aplpinctrl.c,v 1.4 2022/04/06 18:59:26 naddy Exp $	*/
/*
 * Copyright (c) 2021 Mark Kettenis <kettenis@openbsd.org>
 * Copyright (c) 2022 Kyle Evans <kevans@FreeBSD.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>

#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/fdt_pinctrl.h>

#include "pic_if.h"
#include "gpio_if.h"

#define APPLE_PIN(pinmux) ((pinmux) & 0xffff)
#define APPLE_FUNC(pinmux) ((pinmux) >> 16)

#define GPIO_PIN(pin)		((pin) * 4)
#define  GPIO_PIN_GROUP_MASK	(7 << 16)
#define  GPIO_PIN_INPUT_ENABLE	(1 << 9)
#define  GPIO_PIN_GROUP_SHIFT	16
#define  GPIO_PIN_FUNC_MASK	(3 << 5)
#define  GPIO_PIN_FUNC_SHIFT	5
#define  GPIO_PIN_MODE_MASK	(7 << 1)
#define  GPIO_PIN_MODE_HI	(1 << 4)	/* Linux REG_GPIOx_MODE_HI */
#define  GPIO_PIN_MODE_INPUT	(0 << 1)
#define  GPIO_PIN_MODE_OUTPUT	(1 << 1)
#define  GPIO_PIN_MODE_IRQ_HI	(2 << 1)
#define  GPIO_PIN_MODE_IRQ_LO	(3 << 1)
#define  GPIO_PIN_MODE_IRQ_UP	(4 << 1)
#define  GPIO_PIN_MODE_IRQ_DN	(5 << 1)
#define  GPIO_PIN_MODE_IRQ_ANY	(6 << 1)
#define  GPIO_PIN_MODE_IRQ_OFF	(7 << 1)
#define  GPIO_PIN_DATA		(1 << 0)
#define GPIO_IRQ(grp, pin)	(0x800 + (grp) * 64 + ((pin) >> 5) * 4)

#define	APPLE_PINCTRL_DEFAULT_CAPS	\
	(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT)

#define HREAD4(sc, reg)							\
	bus_read_4((sc)->sc_res[APPLE_PINCTRL_MEMRES], reg)
#define HWRITE4(sc, reg, val)						\
	bus_write_4((sc)->sc_res[APPLE_PINCTRL_MEMRES], reg, val)
#define HSET4(sc, reg, bits)						\
	HWRITE4((sc), (reg), HREAD4((sc), (reg)) | (bits))
#define HCLR4(sc, reg, bits)						\
	HWRITE4((sc), (reg), HREAD4((sc), (reg)) & ~(bits))

struct apple_pinctrl_irqsrc {
	struct intr_irqsrc	isrc;
	u_int			pin;
	uint32_t		mode;	/* GPIO_PIN_MODE_IRQ_*, OFF if unused */
	bool			level;
	bool			registered;
};

/* The IRQ group our pins are routed to (Linux irq_group_base, default 0). */
#define	APPLE_PINCTRL_IRQ_GROUP	0

enum {
	APPLE_PINCTRL_MEMRES = 0,
	APPLE_PINCTRL_IRQRES,
	APPLE_PINCTRL_NRES,
};

struct apple_pinctrl_softc {
	device_t		sc_dev;
	device_t		sc_busdev;
	struct mtx		sc_mtx;
	int			sc_ngpios;
	/* gpio-reserved-ranges: pins that are not the AP's (a read can SError). */
	uint8_t			*sc_reserved;

	void			*sc_intrhand;
	struct resource		*sc_res[APPLE_PINCTRL_NRES];
	struct apple_pinctrl_irqsrc	*sc_irqs;
	/*
	 * Pins whose status bits the parent filter may act on: non-reserved
	 * pins with a handler set up and enabled.  One bit per pin.
	 */
	uint32_t		*sc_irq_enabled;
	/* The parent (AIC) filter is installed on first use, from a task. */
	struct task		sc_parent_task;
	bool			sc_parent_queued;
	bool			sc_pic;
};

#define	APPLE_PINCTRL_LOCK(sc)		mtx_lock_spin(&(sc)->sc_mtx)
#define	APPLE_PINCTRL_UNLOCK(sc)	mtx_unlock_spin(&(sc)->sc_mtx)
#define	APPLE_PINCTRL_LOCK_ASSERT(sc)	mtx_assert(&(sc)->sc_mtx, MA_OWNED)

static struct ofw_compat_data compat_data[] = {
	{"apple,pinctrl",	1},
	{"apple,t8103-pinctrl",	1},
	{NULL,			0},
};

/* A pin we may touch: in range and not in gpio-reserved-ranges. */
static bool
apple_pinctrl_pin_ok(struct apple_pinctrl_softc *sc, uint32_t pin)
{

	return (pin < sc->sc_ngpios &&
	    (sc->sc_reserved == NULL || !sc->sc_reserved[pin]));
}

static void
apple_pinctrl_read_reserved(struct apple_pinctrl_softc *sc, phandle_t node)
{
	pcell_t *ranges;
	ssize_t len;
	uint32_t i, pin;

	len = OF_getencprop_alloc_multi(node, "gpio-reserved-ranges",
	    sizeof(*ranges) * 2, (void **)&ranges);
	if (len <= 0)
		return;
	sc->sc_reserved = malloc(sc->sc_ngpios, M_DEVBUF, M_WAITOK | M_ZERO);
	for (i = 0; i < len; i++)
		for (pin = ranges[2 * i]; pin < ranges[2 * i] +
		    ranges[2 * i + 1] && pin < sc->sc_ngpios; pin++)
			sc->sc_reserved[pin] = 1;
	OF_prop_free(ranges);
}

static struct resource_spec apple_pinctrl_res_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_OPTIONAL },
	{ -1,			0,	0 },
};

static int	apple_pinctrl_probe(device_t dev);
static int	apple_pinctrl_attach(device_t dev);
static int	apple_pinctrl_detach(device_t dev);

static void	apple_pinctrl_pic_attach(struct apple_pinctrl_softc *, phandle_t);
static int	apple_pinctrl_configure(device_t, phandle_t);
static phandle_t	apple_pinctrl_get_node(device_t, device_t);

static int
apple_pinctrl_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Apple Pinmux Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_pinctrl_attach(device_t dev)
{
	pcell_t gpio_ranges[4];
	phandle_t node;
	struct apple_pinctrl_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	node = ofw_bus_get_node(dev);

	if (bus_alloc_resources(dev, apple_pinctrl_res_spec, sc->sc_res) != 0) {
		device_printf(dev, "cannot allocate device resources\n");
		return (ENXIO);
	}

	mtx_init(&sc->sc_mtx, "aapl gpio", "gpio", MTX_SPIN);

	error = OF_getencprop(node, "gpio-ranges", gpio_ranges,
	    sizeof(gpio_ranges));
	if (error == -1) {
		device_printf(dev, "failed to get gpio-ranges\n");
		goto error;
	}

	sc->sc_ngpios = gpio_ranges[3];
	if (sc->sc_ngpios == 0) {
		device_printf(dev, "no GPIOs\n");
		goto error;
	}

	apple_pinctrl_read_reserved(sc, node);

	fdt_pinctrl_register(dev, "pinmux");
	fdt_pinctrl_configure_tree(dev);

	if (OF_hasprop(node, "interrupt-controller") &&
	    sc->sc_res[APPLE_PINCTRL_IRQRES] != NULL)
		apple_pinctrl_pic_attach(sc, node);

	sc->sc_busdev = gpiobus_add_bus(dev);
	if (sc->sc_busdev == NULL) {
		device_printf(dev, "failed to attach gpiobus\n");
		goto error;
	}

	bus_attach_children(dev);
	return (0);
error:
	mtx_destroy(&sc->sc_mtx);
	bus_release_resources(dev, apple_pinctrl_res_spec, sc->sc_res);
	return (ENXIO);
}

static int
apple_pinctrl_detach(device_t dev)
{

	return (EBUSY);
}

static void
apple_pinctrl_pin_configure(struct apple_pinctrl_softc *sc, uint32_t pin,
    uint32_t flags)
{
	uint32_t reg;

	APPLE_PINCTRL_LOCK_ASSERT(sc);

	MPASS(pin < sc->sc_ngpios);

	reg = HREAD4(sc, GPIO_PIN(pin));
	reg &= ~GPIO_PIN_FUNC_MASK;
	reg &= ~GPIO_PIN_MODE_MASK;

	if ((flags & GPIO_PIN_PRESET_LOW) != 0)
		reg &= ~GPIO_PIN_DATA;
	else if ((flags & GPIO_PIN_PRESET_HIGH) != 0)
		reg |= GPIO_PIN_DATA;

	if ((flags & GPIO_PIN_INPUT) != 0)
		reg |= GPIO_PIN_MODE_INPUT;
	else if ((flags & GPIO_PIN_OUTPUT) != 0)
		reg |= GPIO_PIN_MODE_OUTPUT;

	HWRITE4(sc, GPIO_PIN(pin), reg);
}

static device_t
apple_pinctrl_get_bus(device_t dev)
{
	struct apple_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	return (sc->sc_busdev);
}

static int
apple_pinctrl_pin_max(device_t dev, int *maxpin)
{
	struct apple_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	*maxpin = sc->sc_ngpios - 1;
	return (0);
}

static int
apple_pinctrl_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct apple_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (pin >= sc->sc_ngpios)
		return (EINVAL);

	snprintf(name, GPIOMAXNAME - 1, "gpio%c%d",
	    device_get_unit(dev) + 'a', pin);

	return (0);
}

static int
apple_pinctrl_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct apple_pinctrl_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	if (!apple_pinctrl_pin_ok(sc, pin))
		return (EINVAL);

	*flags = 0;

	APPLE_PINCTRL_LOCK(sc);

	reg = HREAD4(sc, GPIO_PIN(pin));
	if ((reg & GPIO_PIN_FUNC_MASK) == 0) {
		if ((reg & GPIO_PIN_MODE_MASK) == GPIO_PIN_MODE_OUTPUT)
			*flags |= GPIO_PIN_OUTPUT;
		else
			*flags |= GPIO_PIN_INPUT;
	}

	APPLE_PINCTRL_UNLOCK(sc);

	return (0);
}

static int
apple_pinctrl_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{

	*caps = APPLE_PINCTRL_DEFAULT_CAPS;
	return (0);
}

static int
apple_pinctrl_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct apple_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (!apple_pinctrl_pin_ok(sc, pin))
		return (EINVAL);

	APPLE_PINCTRL_LOCK(sc);
	apple_pinctrl_pin_configure(sc, pin, flags);
	APPLE_PINCTRL_UNLOCK(sc);

	return (0);
}

static int
apple_pinctrl_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct apple_pinctrl_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	if (!apple_pinctrl_pin_ok(sc, pin))
		return (EINVAL);

	APPLE_PINCTRL_LOCK(sc);
	reg = HREAD4(sc, GPIO_PIN(pin));
	*val = !!(reg & GPIO_PIN_DATA);
	APPLE_PINCTRL_UNLOCK(sc);

	return (0);
}

static int
apple_pinctrl_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct apple_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (!apple_pinctrl_pin_ok(sc, pin))
		return (EINVAL);

	APPLE_PINCTRL_LOCK(sc);
	if (value)
		HSET4(sc, GPIO_PIN(pin), GPIO_PIN_DATA);
	else
		HCLR4(sc, GPIO_PIN(pin), GPIO_PIN_DATA);
	APPLE_PINCTRL_UNLOCK(sc);
	return (0);
}


static int
apple_pinctrl_pin_toggle(device_t dev, uint32_t pin)
{
	struct apple_pinctrl_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	if (!apple_pinctrl_pin_ok(sc, pin))
		return (EINVAL);

	APPLE_PINCTRL_LOCK(sc);
	reg = HREAD4(sc, GPIO_PIN(pin));
	if ((reg & GPIO_PIN_DATA) == 0)
		reg |= GPIO_PIN_DATA;
	else
		reg &= ~GPIO_PIN_DATA;
	HWRITE4(sc, GPIO_PIN(pin), reg);
	APPLE_PINCTRL_UNLOCK(sc);
	return (0);
}


static int
apple_pinctrl_pin_config_32(device_t dev, uint32_t first_pin, uint32_t num_pins,
    uint32_t *pin_flags)
{
	struct apple_pinctrl_softc *sc;
	uint32_t pin;

	sc = device_get_softc(dev);
	if (first_pin >= sc->sc_ngpios)
		return (EINVAL);

	/*
	 * The configuration for a bank of pins is scattered among several
	 * registers; we cannot g'tee to simultaneously change the state of all
	 * the pins in the flags array.  So just loop through the array
	 * configuring each pin for now.  If there was a strong need, it might
	 * be possible to support some limited simultaneous config, such as
	 * adjacent groups of 8 pins that line up the same as the config regs.
	 */
	APPLE_PINCTRL_LOCK(sc);
	for (pin = 0; pin < num_pins; ++pin) {
		if (apple_pinctrl_pin_ok(sc, first_pin + pin) &&
		    (pin_flags[pin] & (GPIO_PIN_INPUT | GPIO_PIN_OUTPUT)))
			apple_pinctrl_pin_configure(sc, first_pin + pin,
			    pin_flags[pin]);
	}
	APPLE_PINCTRL_UNLOCK(sc);

	return (0);
}

static phandle_t
apple_pinctrl_get_node(device_t dev, device_t bus)
{

	/* GPIO bus */
	return (ofw_bus_get_node(dev));
}

static int
apple_pinctrl_configure(device_t dev, phandle_t cfgxref)
{
	struct apple_pinctrl_softc *sc;
	pcell_t *pinmux;
	phandle_t node;
	ssize_t len;
	uint32_t reg;
	uint16_t pin, func;
	int i;

	sc = device_get_softc(dev);
	node = OF_node_from_xref(cfgxref);

	len = OF_getencprop_alloc(node, "pinmux", (void **)&pinmux);
	if (len <= 0)
		return (-1);

	APPLE_PINCTRL_LOCK(sc);
	for (i = 0; i < len / sizeof(pcell_t); i++) {
		pin = APPLE_PIN(pinmux[i]);
		func = APPLE_FUNC(pinmux[i]);
		if (!apple_pinctrl_pin_ok(sc, pin))
			continue;
		reg = HREAD4(sc, GPIO_PIN(pin));
		reg &= ~GPIO_PIN_FUNC_MASK;
		reg |= (func << GPIO_PIN_FUNC_SHIFT) & GPIO_PIN_FUNC_MASK;
		HWRITE4(sc, GPIO_PIN(pin), reg);
	}
	APPLE_PINCTRL_UNLOCK(sc);

	OF_prop_free(pinmux);
	return 0;
}

/*
 * Interrupt controller, after Linux pinctrl-apple-gpio.c.
 *
 * Each pin register carries the pin's IRQ mode (GPIO_PIN_MODE_IRQ_*, OFF
 * when unused) and the IRQ group it reports to; each group has a block of
 * write-one-to-clear status registers at GPIO_IRQ(grp, pin) and its own
 * parent (AIC) interrupt.  Like Linux we route every pin to one group
 * (APPLE_PINCTRL_IRQ_GROUP) and so use only that group's parent interrupt.
 *
 * Pins in gpio-reserved-ranges belong to other agents: their pin registers
 * are never touched (a read can SError), they get no interrupt source, and
 * their status bits are neither acted on nor cleared.  The parent filter
 * only looks at bits in sc_irq_enabled.  The parent filter itself is only
 * installed when a consumer first sets up a pin interrupt, so a kernel
 * without consumers behaves exactly as before.
 */

#define	APPLE_PINCTRL_IRQ_MODE_MASK	(GPIO_PIN_MODE_MASK | GPIO_PIN_MODE_HI)

static void
apple_pinctrl_irq_set_mode(struct apple_pinctrl_softc *sc, u_int pin,
    uint32_t mode)
{
	uint32_t reg;

	APPLE_PINCTRL_LOCK_ASSERT(sc);
	MPASS(apple_pinctrl_pin_ok(sc, pin));

	reg = HREAD4(sc, GPIO_PIN(pin));
	reg &= ~APPLE_PINCTRL_IRQ_MODE_MASK;
	reg |= mode;
	HWRITE4(sc, GPIO_PIN(pin), reg);
}

static void
apple_pinctrl_irq_ack(struct apple_pinctrl_softc *sc, u_int pin)
{

	MPASS(apple_pinctrl_pin_ok(sc, pin));
	HWRITE4(sc, GPIO_IRQ(APPLE_PINCTRL_IRQ_GROUP, pin), 1u << (pin % 32));
}

static int
apple_pinctrl_intr(void *arg)
{
	struct apple_pinctrl_softc *sc = arg;
	struct apple_pinctrl_irqsrc *pi;
	struct trapframe *tf = curthread->td_intr_frame;
	uint32_t pending;
	u_int pin, word;

	for (word = 0; word < howmany(sc->sc_ngpios, 32); word++) {
		if (atomic_load_32(&sc->sc_irq_enabled[word]) == 0)
			continue;
		pending = HREAD4(sc, GPIO_IRQ(APPLE_PINCTRL_IRQ_GROUP,
		    word * 32));
		/* Only bits of our pins with a handler. */
		pending &= atomic_load_32(&sc->sc_irq_enabled[word]);
		while (pending != 0) {
			pin = word * 32 + ffs(pending) - 1;
			pending &= pending - 1;
			pi = &sc->sc_irqs[pin];
			/*
			 * Clear the latch before dispatch: a level source
			 * that is still asserted latches again (and level
			 * sources with an ithread are masked in pre_ithread).
			 */
			apple_pinctrl_irq_ack(sc, pin);
			if (intr_isrc_dispatch(&pi->isrc, tf) != 0) {
				APPLE_PINCTRL_LOCK(sc);
				atomic_clear_32(&sc->sc_irq_enabled[word],
				    1u << (pin % 32));
				apple_pinctrl_irq_set_mode(sc, pin,
				    GPIO_PIN_MODE_IRQ_OFF);
				APPLE_PINCTRL_UNLOCK(sc);
				device_printf(sc->sc_dev,
				    "stray interrupt on pin %u, disabled\n", pin);
			}
		}
	}
	return (FILTER_HANDLED);
}

static void
apple_pinctrl_parent_task(void *arg, int pending __unused)
{
	struct apple_pinctrl_softc *sc = arg;
	int error;

	if (sc->sc_intrhand != NULL)
		return;
	error = bus_setup_intr(sc->sc_dev, sc->sc_res[APPLE_PINCTRL_IRQRES],
	    INTR_TYPE_MISC | INTR_MPSAFE, apple_pinctrl_intr, NULL, sc,
	    &sc->sc_intrhand);
	if (error != 0) {
		sc->sc_intrhand = NULL;
		device_printf(sc->sc_dev,
		    "cannot set up parent interrupt: %d\n", error);
	}
}

static void
apple_pinctrl_pic_attach(struct apple_pinctrl_softc *sc, phandle_t node)
{
	struct apple_pinctrl_irqsrc *pi;
	const char *name;
	uint32_t reg, mode;
	u_int pin;
	int error;

	name = device_get_nameunit(sc->sc_dev);
	sc->sc_irqs = malloc(sizeof(*sc->sc_irqs) * sc->sc_ngpios, M_DEVBUF,
	    M_WAITOK | M_ZERO);
	sc->sc_irq_enabled = malloc(sizeof(uint32_t) *
	    howmany(sc->sc_ngpios, 32), M_DEVBUF, M_WAITOK | M_ZERO);
	TASK_INIT(&sc->sc_parent_task, 0, apple_pinctrl_parent_task, sc);

	for (pin = 0; pin < sc->sc_ngpios; pin++) {
		if (!apple_pinctrl_pin_ok(sc, pin))
			continue;
		pi = &sc->sc_irqs[pin];
		pi->pin = pin;
		pi->mode = GPIO_PIN_MODE_IRQ_OFF;

		/* No consumer yet: any IRQ mode left by the loader goes OFF. */
		APPLE_PINCTRL_LOCK(sc);
		reg = HREAD4(sc, GPIO_PIN(pin));
		mode = reg & GPIO_PIN_MODE_MASK;
		if ((reg & GPIO_PIN_FUNC_MASK) == 0 &&
		    mode >= GPIO_PIN_MODE_IRQ_HI &&
		    mode <= GPIO_PIN_MODE_IRQ_ANY)
			apple_pinctrl_irq_set_mode(sc, pin,
			    GPIO_PIN_MODE_IRQ_OFF);
		APPLE_PINCTRL_UNLOCK(sc);

		error = intr_isrc_register(&pi->isrc, sc->sc_dev, 0, "%s,%u",
		    name, pin);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "cannot register interrupt source %u: %d\n", pin,
			    error);
			continue;
		}
		pi->registered = true;
	}

	if (intr_pic_register(sc->sc_dev, OF_xref_from_node(node)) == NULL) {
		device_printf(sc->sc_dev, "cannot register PIC\n");
		return;
	}
	sc->sc_pic = true;
}

static int
apple_pinctrl_map(struct apple_pinctrl_softc *sc, struct intr_map_data *data,
    u_int *pinp, uint32_t *modep, bool *levelp)
{
	struct intr_map_data_fdt *daf;
	struct intr_map_data_gpio *dag;
	uint32_t mode;
	u_int pin;
	bool level;

	switch (data->type) {
	case INTR_MAP_DATA_FDT:
		daf = (struct intr_map_data_fdt *)data;
		if (daf->ncells != 2)
			return (EINVAL);
		pin = daf->cells[0];
		/* dt-bindings/interrupt-controller/irq.h IRQ_TYPE_* */
		switch (daf->cells[1] & 0xf) {
		case 1:
			mode = GPIO_PIN_MODE_IRQ_UP;
			level = false;
			break;
		case 2:
			mode = GPIO_PIN_MODE_IRQ_DN;
			level = false;
			break;
		case 3:
			mode = GPIO_PIN_MODE_IRQ_ANY;
			level = false;
			break;
		case 4:
			mode = GPIO_PIN_MODE_IRQ_HI;
			level = true;
			break;
		case 8:
			mode = GPIO_PIN_MODE_IRQ_LO;
			level = true;
			break;
		default:
			return (EINVAL);
		}
		break;
	case INTR_MAP_DATA_GPIO:
		dag = (struct intr_map_data_gpio *)data;
		pin = dag->gpio_pin_num;
		switch (dag->gpio_intr_mode & GPIO_INTR_MASK) {
		case GPIO_INTR_EDGE_RISING:
			mode = GPIO_PIN_MODE_IRQ_UP;
			level = false;
			break;
		case GPIO_INTR_EDGE_FALLING:
			mode = GPIO_PIN_MODE_IRQ_DN;
			level = false;
			break;
		case GPIO_INTR_EDGE_BOTH:
			mode = GPIO_PIN_MODE_IRQ_ANY;
			level = false;
			break;
		case GPIO_INTR_LEVEL_HIGH:
			mode = GPIO_PIN_MODE_IRQ_HI;
			level = true;
			break;
		case GPIO_INTR_LEVEL_LOW:
			mode = GPIO_PIN_MODE_IRQ_LO;
			level = true;
			break;
		default:
			return (EINVAL);
		}
		break;
	default:
		return (ENOTSUP);
	}

	/* Never hand out a reserved pin. */
	if (!apple_pinctrl_pin_ok(sc, pin) || !sc->sc_irqs[pin].registered)
		return (EINVAL);
	*pinp = pin;
	if (modep != NULL)
		*modep = mode;
	if (levelp != NULL)
		*levelp = level;
	return (0);
}

static int
apple_pinctrl_pic_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	struct apple_pinctrl_softc *sc = device_get_softc(dev);
	u_int pin;
	int error;

	error = apple_pinctrl_map(sc, data, &pin, NULL, NULL);
	if (error == 0)
		*isrcp = &sc->sc_irqs[pin].isrc;
	return (error);
}

static int
apple_pinctrl_pic_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct apple_pinctrl_softc *sc = device_get_softc(dev);
	struct apple_pinctrl_irqsrc *pi = (struct apple_pinctrl_irqsrc *)isrc;
	uint32_t mode, reg;
	u_int pin;
	bool level;

	if (data == NULL)
		return (ENOTSUP);
	if (apple_pinctrl_map(sc, data, &pin, &mode, &level) != 0)
		return (EINVAL);
	if (pi->pin != pin)
		return (EINVAL);
	if (isrc->isrc_handlers != 0)
		return (pi->mode == mode ? 0 : EINVAL);

	pi->mode = mode;
	pi->level = level;

	/*
	 * Linux irq_startup: route to our group, GPIO function, input
	 * enabled, IRQ still off (enable_intr turns it on).
	 */
	APPLE_PINCTRL_LOCK(sc);
	reg = HREAD4(sc, GPIO_PIN(pin));
	reg &= ~(GPIO_PIN_GROUP_MASK | GPIO_PIN_FUNC_MASK |
	    APPLE_PINCTRL_IRQ_MODE_MASK);
	reg |= (APPLE_PINCTRL_IRQ_GROUP << GPIO_PIN_GROUP_SHIFT) &
	    GPIO_PIN_GROUP_MASK;
	reg |= GPIO_PIN_INPUT_ENABLE | GPIO_PIN_MODE_IRQ_OFF;
	HWRITE4(sc, GPIO_PIN(pin), reg);
	apple_pinctrl_irq_ack(sc, pin);
	APPLE_PINCTRL_UNLOCK(sc);

	/*
	 * bus_setup_intr can sleep and we hold the isrc table lock (a
	 * default mutex, so enqueueing is fine; isrc_table_lock also
	 * serialises sc_parent_queued).
	 */
	if (!sc->sc_parent_queued) {
		sc->sc_parent_queued = true;
		taskqueue_enqueue(taskqueue_thread, &sc->sc_parent_task);
	}
	return (0);
}

static int
apple_pinctrl_pic_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct apple_pinctrl_softc *sc = device_get_softc(dev);
	struct apple_pinctrl_irqsrc *pi = (struct apple_pinctrl_irqsrc *)isrc;

	if (isrc->isrc_handlers == 0) {
		APPLE_PINCTRL_LOCK(sc);
		atomic_clear_32(&sc->sc_irq_enabled[pi->pin / 32],
		    1u << (pi->pin % 32));
		apple_pinctrl_irq_set_mode(sc, pi->pin, GPIO_PIN_MODE_IRQ_OFF);
		apple_pinctrl_irq_ack(sc, pi->pin);
		pi->mode = GPIO_PIN_MODE_IRQ_OFF;
		APPLE_PINCTRL_UNLOCK(sc);
	}
	return (0);
}

static void
apple_pinctrl_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct apple_pinctrl_softc *sc = device_get_softc(dev);
	struct apple_pinctrl_irqsrc *pi = (struct apple_pinctrl_irqsrc *)isrc;

	APPLE_PINCTRL_LOCK(sc);
	atomic_set_32(&sc->sc_irq_enabled[pi->pin / 32], 1u << (pi->pin % 32));
	apple_pinctrl_irq_set_mode(sc, pi->pin, pi->mode);
	APPLE_PINCTRL_UNLOCK(sc);
}

static void
apple_pinctrl_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct apple_pinctrl_softc *sc = device_get_softc(dev);
	struct apple_pinctrl_irqsrc *pi = (struct apple_pinctrl_irqsrc *)isrc;

	APPLE_PINCTRL_LOCK(sc);
	atomic_clear_32(&sc->sc_irq_enabled[pi->pin / 32],
	    1u << (pi->pin % 32));
	apple_pinctrl_irq_set_mode(sc, pi->pin, GPIO_PIN_MODE_IRQ_OFF);
	APPLE_PINCTRL_UNLOCK(sc);
}

static void
apple_pinctrl_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	struct apple_pinctrl_softc *sc = device_get_softc(dev);
	struct apple_pinctrl_irqsrc *pi = (struct apple_pinctrl_irqsrc *)isrc;

	/* Linux handle_level_irq: mask and ack until the handler is done. */
	if (!pi->level)
		return;
	APPLE_PINCTRL_LOCK(sc);
	apple_pinctrl_irq_set_mode(sc, pi->pin, GPIO_PIN_MODE_IRQ_OFF);
	apple_pinctrl_irq_ack(sc, pi->pin);
	APPLE_PINCTRL_UNLOCK(sc);
}

static void
apple_pinctrl_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	struct apple_pinctrl_softc *sc = device_get_softc(dev);
	struct apple_pinctrl_irqsrc *pi = (struct apple_pinctrl_irqsrc *)isrc;

	if (!pi->level)
		return;
	APPLE_PINCTRL_LOCK(sc);
	/* Not if the source was disabled meanwhile. */
	if ((atomic_load_32(&sc->sc_irq_enabled[pi->pin / 32]) &
	    (1u << (pi->pin % 32))) != 0)
		apple_pinctrl_irq_set_mode(sc, pi->pin, pi->mode);
	APPLE_PINCTRL_UNLOCK(sc);
}

static void
apple_pinctrl_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{

	/* The latch was cleared before dispatch. */
}

static device_method_t apple_pinctrl_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		apple_pinctrl_probe),
	DEVMETHOD(device_attach,	apple_pinctrl_attach),
	DEVMETHOD(device_detach,	apple_pinctrl_detach),

	/* GPIO protocol */
	DEVMETHOD(gpio_get_bus,		apple_pinctrl_get_bus),
	DEVMETHOD(gpio_pin_max,		apple_pinctrl_pin_max),
	DEVMETHOD(gpio_pin_getname,	apple_pinctrl_pin_getname),
	DEVMETHOD(gpio_pin_getflags,	apple_pinctrl_pin_getflags),
	DEVMETHOD(gpio_pin_getcaps,	apple_pinctrl_pin_getcaps),
	DEVMETHOD(gpio_pin_setflags,	apple_pinctrl_pin_setflags),
	DEVMETHOD(gpio_pin_get,		apple_pinctrl_pin_get),
	DEVMETHOD(gpio_pin_set,		apple_pinctrl_pin_set),
	DEVMETHOD(gpio_pin_toggle,	apple_pinctrl_pin_toggle),
	DEVMETHOD(gpio_pin_config_32,	apple_pinctrl_pin_config_32),

	/* Interrupt controller interface */
	DEVMETHOD(pic_map_intr,		apple_pinctrl_pic_map_intr),
	DEVMETHOD(pic_setup_intr,	apple_pinctrl_pic_setup_intr),
	DEVMETHOD(pic_teardown_intr,	apple_pinctrl_pic_teardown_intr),
	DEVMETHOD(pic_enable_intr,	apple_pinctrl_pic_enable_intr),
	DEVMETHOD(pic_disable_intr,	apple_pinctrl_pic_disable_intr),
	DEVMETHOD(pic_pre_ithread,	apple_pinctrl_pic_pre_ithread),
	DEVMETHOD(pic_post_ithread,	apple_pinctrl_pic_post_ithread),
	DEVMETHOD(pic_post_filter,	apple_pinctrl_pic_post_filter),

	/* ofw_bus interface */
	DEVMETHOD(ofw_bus_get_node,		apple_pinctrl_get_node),

        /* fdt_pinctrl interface */
	DEVMETHOD(fdt_pinctrl_configure,	apple_pinctrl_configure),

	DEVMETHOD_END
};

static driver_t apple_pinctrl_driver = {
	"gpio",
	apple_pinctrl_methods,
	sizeof(struct apple_pinctrl_softc),
};

EARLY_DRIVER_MODULE(apple_pinctrl, simplebus, apple_pinctrl_driver,
    0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
