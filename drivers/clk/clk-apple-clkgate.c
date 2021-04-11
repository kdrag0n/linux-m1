// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple SoC clock/power gating driver
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>

#define APPLE_CLOCK_TARGET_MODE GENMASK(3, 0)
#define APPLE_CLOCK_ACTUAL_MODE GENMASK(7, 4)

#define APPLE_CLOCK_ENABLE 0xf
#define APPLE_CLOCK_DISABLE 0x0

#define APPLE_CLOCK_ENDISABLE_TIMEOUT 100

struct apple_clk_gate {
	struct clk_hw hw;
	struct regmap *regmap;
	u32 offset;
};

#define to_apple_clk_gate(_hw) container_of(_hw, struct apple_clk_gate, hw)

static int apple_clk_gate_endisable(struct clk_hw *hw, int enable)
{
	struct apple_clk_gate *gate = to_apple_clk_gate(hw);
	u32 reg;
	u32 mode;

	if (enable)
		mode = APPLE_CLOCK_ENABLE;
	else
		mode = APPLE_CLOCK_DISABLE;

	printk("clk-gate: 0x%x -> %d\n", gate->offset, enable);

	regmap_update_bits(gate->regmap, gate->offset, APPLE_CLOCK_TARGET_MODE,
			   FIELD_PREP(APPLE_CLOCK_TARGET_MODE, mode));

	return regmap_read_poll_timeout_atomic(
		gate->regmap, gate->offset, reg,
		(FIELD_GET(APPLE_CLOCK_ACTUAL_MODE, reg) == mode), 1,
		APPLE_CLOCK_ENDISABLE_TIMEOUT);
}

static int apple_clk_gate_enable(struct clk_hw *hw)
{
	return apple_clk_gate_endisable(hw, 1);
}

static void apple_clk_gate_disable(struct clk_hw *hw)
{
	apple_clk_gate_endisable(hw, 0);
}

static int apple_clk_gate_is_enabled(struct clk_hw *hw)
{
	struct apple_clk_gate *gate = to_apple_clk_gate(hw);
	u32 reg;

	regmap_read(gate->regmap, gate->offset, &reg);
	return FIELD_GET(APPLE_CLOCK_ACTUAL_MODE, reg) == APPLE_CLOCK_ENABLE;
}

static const struct clk_ops apple_clk_gate_ops = {
	.enable = apple_clk_gate_enable,
	.disable = apple_clk_gate_disable,
	.is_enabled = apple_clk_gate_is_enabled,
};

static int apple_clk_gate_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct apple_clk_gate *gate;
	struct clk_hw *hw;
	struct clk_init_data init;
	struct regmap *regmap;
	int ret;
	struct clk_parent_data parent_data[1] = {
		{.index = 0},
	};

	regmap = syscon_node_to_regmap(node->parent);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	memset(&init, 0, sizeof(init));
	gate = devm_kzalloc(dev, sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return -ENOMEM;

	gate->regmap = regmap;
	hw = &gate->hw;
	hw->init = &init;

	ret = of_property_read_string(node, "clock-output-names", &init.name);
	if (ret < 0) {
		dev_err(dev, "missing name property\n");
		return ret;
	}

	ret = of_property_read_u32(node, "reg", &gate->offset);
	if (ret < 0) {
		dev_err(dev, "missing reg property\n");
		return ret;
	}

	if (of_clk_get_parent_count(node) != 1) {
		dev_err(dev, "expected exactly one clock parent\n");
		return ret;
	}

	init.ops = &apple_clk_gate_ops;
	init.flags = 0;
	init.parent_names = NULL;
	init.parent_data = parent_data;
	init.num_parents = 1;

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, hw);
	if (ret < 0)
		return ret;

	ret = devm_clk_hw_register(dev, hw);
	if (ret) {
		dev_err(dev, "failed to register clock\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id apple_clk_gate_of_match[] = {
	{ .compatible = "apple,t8103-clock-gate" },
	{ .compatible = "apple,clock-gate" },
	{}
};

MODULE_DEVICE_TABLE(of, apple_clk_gate_of_match);

static struct platform_driver apple_clk_gate_driver = {
	.probe = apple_clk_gate_probe,
	.driver = {
		.name = "apple-clock-gate",
		.of_match_table = apple_clk_gate_of_match,
	},
};

MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_DESCRIPTION("Clock gating driver for Apple SoCs");
MODULE_LICENSE("GPL v2");

module_platform_driver(apple_clk_gate_driver);
