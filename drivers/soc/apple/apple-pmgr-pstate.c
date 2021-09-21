// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple SoC PMGR device power state driver
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>

#define APPLE_PMGR_RESET     BIT(31)
#define APPLE_PMGR_PS_TARGET GENMASK(3, 0)
#define APPLE_PMGR_PS_ACTUAL GENMASK(7, 4)

#define APPLE_PMGR_PS_ACTIVE  0xf
#define APPLE_PMGR_PS_CLKGATE 0x4
#define APPLE_PMGR_PS_PWRGATE 0x0

#define APPLE_PMGR_PS_SET_TIMEOUT 100

struct apple_pmgr_ps {
	struct generic_pm_domain genpd;
	struct regmap *regmap;
	u32 offset;
};

#define to_apple_pmgr_ps(_genpd) container_of(_genpd, struct apple_pmgr_ps, genpd)

static int apple_pmgr_ps_set(struct generic_pm_domain *genpd, u32 pstate)
{
	struct apple_pmgr_ps *ps = to_apple_pmgr_ps(genpd);
	u32 reg;

	printk("clk-ps: 0x%x -> 0x%x\n", ps->offset, pstate);

	regmap_update_bits(ps->regmap, ps->offset, APPLE_PMGR_PS_TARGET,
			   FIELD_PREP(APPLE_PMGR_PS_TARGET, pstate));

	return regmap_read_poll_timeout_atomic(
		ps->regmap, ps->offset, reg,
		(FIELD_GET(APPLE_PMGR_PS_ACTUAL, reg) == pstate), 1,
		APPLE_PMGR_PS_SET_TIMEOUT);
}

static bool apple_pmgr_ps_is_active(struct apple_pmgr_ps *ps)
{
	u32 reg;

	regmap_read(ps->regmap, ps->offset, &reg);
	return FIELD_GET(APPLE_PMGR_PS_ACTUAL, reg) == APPLE_PMGR_PS_ACTIVE;
}

static int apple_pmgr_ps_power_on(struct generic_pm_domain *genpd)
{
	return apple_pmgr_ps_set(genpd, APPLE_PMGR_PS_ACTIVE);
}

static int apple_pmgr_ps_power_off(struct generic_pm_domain *genpd)
{
	return apple_pmgr_ps_set(genpd, APPLE_PMGR_PS_PWRGATE);
}

static int apple_pmgr_ps_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct apple_pmgr_ps *ps;
	struct regmap *regmap;
	struct of_phandle_iterator it;
	int ret;
	const char *name;

	regmap = syscon_node_to_regmap(node->parent);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ps = devm_kzalloc(dev, sizeof(*ps), GFP_KERNEL);
	if (!ps)
		return -ENOMEM;

	ps->regmap = regmap;

	ret = of_property_read_string(node, "domain-output-names", &name);
	if (ret < 0) {
		dev_err(dev, "missing domain-output-names property\n");
		return ret;
	}

	ret = of_property_read_u32(node, "reg", &ps->offset);
	if (ret < 0) {
		dev_err(dev, "missing reg property\n");
		return ret;
	}

	if (of_property_read_bool(node, "always-on"))
		ps->genpd.flags |= GENPD_FLAG_ALWAYS_ON;

	ps->genpd.name = name;
	ps->genpd.power_on = apple_pmgr_ps_power_on;
	ps->genpd.power_off = apple_pmgr_ps_power_off;

	ret = pm_genpd_init(&ps->genpd, NULL, !apple_pmgr_ps_is_active(ps));
	if (ret < 0) {
		dev_err(dev, "pm_genpd_init failed\n");
		return ret;
	}

	ret = of_genpd_add_provider_simple(node, &ps->genpd);
	if (ret < 0) {
		dev_err(dev, "of_genpd_add_provider_simple failed\n");
		return ret;
	}

	of_for_each_phandle(&it, ret, node, "power-domains", "#power-domain-cells", -1) {
		struct of_phandle_args parent, child;

		parent.np = it.node;
		parent.args_count = of_phandle_iterator_args(&it, parent.args, MAX_PHANDLE_ARGS);
		child.np = node;
		child.args_count = 0;
		ret = of_genpd_add_subdomain(&parent, &child);

		if (ret == -EPROBE_DEFER) {
			of_node_put(parent.np);
			goto err_remove;
		} else if (ret < 0) {
			dev_err(dev, "failed to add to parent domain: %d (%s -> %s)\n",
				ret, it.node->name, node->name);
			of_node_put(parent.np);
			goto err_remove;
		}
	}

	pm_genpd_remove_device(dev);

	return 0;
err_remove:
	of_genpd_del_provider(node);
	pm_genpd_remove(&ps->genpd);
	return ret;
}

static const struct of_device_id apple_pmgr_ps_of_match[] = {
	{ .compatible = "apple,t8103-pmgr-pstate" },
	{ .compatible = "apple,pmgr-pstate" },
	{}
};

MODULE_DEVICE_TABLE(of, apple_pmgr_ps_of_match);

static struct platform_driver apple_pmgr_ps_driver = {
	.probe = apple_pmgr_ps_probe,
	.driver = {
		.name = "apple-pmgr-pstate",
		.of_match_table = apple_pmgr_ps_of_match,
	},
};

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_DESCRIPTION("PMGR power state driver for Apple SoCs");
MODULE_LICENSE("GPL v2");

module_platform_driver(apple_pmgr_ps_driver);
