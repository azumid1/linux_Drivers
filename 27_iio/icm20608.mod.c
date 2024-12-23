#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xfa985410, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x87a740ad, __VMLINUX_SYMBOL_STR(driver_unregister) },
	{ 0x34ce7e95, __VMLINUX_SYMBOL_STR(spi_register_driver) },
	{ 0x63961a83, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0xfbe0f876, __VMLINUX_SYMBOL_STR(spi_setup) },
	{ 0x67b52003, __VMLINUX_SYMBOL_STR(regmap_init_spi) },
	{ 0xf3bb59b5, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x12b8611, __VMLINUX_SYMBOL_STR(regmap_read) },
	{ 0xfef3f0ab, __VMLINUX_SYMBOL_STR(regmap_write) },
	{ 0x8e865d3c, __VMLINUX_SYMBOL_STR(arm_delay_ops) },
	{ 0xefd6cf06, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr0) },
	{ 0x5bc6c4e1, __VMLINUX_SYMBOL_STR(regmap_exit) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "083A3586AB7A9E7CE2AFF09");
