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
	{ 0x18e90764, __VMLINUX_SYMBOL_STR(blk_cleanup_queue) },
	{ 0x97726489, __VMLINUX_SYMBOL_STR(del_gendisk) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0xb5a459dc, __VMLINUX_SYMBOL_STR(unregister_blkdev) },
	{ 0x967825dd, __VMLINUX_SYMBOL_STR(add_disk) },
	{ 0xf47339a5, __VMLINUX_SYMBOL_STR(put_disk) },
	{ 0x715d0819, __VMLINUX_SYMBOL_STR(blk_init_queue) },
	{ 0xba02da45, __VMLINUX_SYMBOL_STR(alloc_disk) },
	{ 0x71a50dbc, __VMLINUX_SYMBOL_STR(register_blkdev) },
	{ 0x76f6c5ef, __VMLINUX_SYMBOL_STR(kmalloc_order) },
	{ 0x3b74bbcf, __VMLINUX_SYMBOL_STR(__blk_end_request_cur) },
	{ 0x9d669763, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0xdc9ee944, __VMLINUX_SYMBOL_STR(page_address) },
	{ 0x3767ef8, __VMLINUX_SYMBOL_STR(blk_fetch_request) },
	{ 0xefd6cf06, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr0) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "31A417BB7719D9D539974B5");
