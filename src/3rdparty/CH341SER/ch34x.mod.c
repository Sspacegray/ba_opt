#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x8498f7ee, "usb_submit_urb" },
	{ 0x76f50aa3, "usb_serial_port_softint" },
	{ 0xa14c5f46, "__dynamic_dev_dbg" },
	{ 0x5bd0f510, "_dev_err" },
	{ 0xe2964344, "__wake_up" },
	{ 0x8dc64d3f, "usb_control_msg" },
	{ 0x527acf24, "tty_buffer_request_room" },
	{ 0x1fe9f97b, "__tty_insert_flip_string_flags" },
	{ 0xd7cbd47b, "tty_flip_buffer_push" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0xfa8a5ed5, "usb_kill_urb" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0xcd9c13a3, "tty_termios_hw_change" },
	{ 0xbd394d8, "tty_termios_baud_rate" },
	{ 0x1f29bb1c, "tty_encode_baud_rate" },
	{ 0x486b45ed, "usb_clear_halt" },
	{ 0x4c03a563, "random_kmalloc_seed" },
	{ 0x24980310, "kmalloc_caches" },
	{ 0x1d199deb, "kmalloc_trace" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x37a0cba, "kfree" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xdeefc704, "usb_serial_register_drivers" },
	{ 0xca035d3, "usb_serial_deregister_drivers" },
	{ 0x2b6e2cf, "pcpu_hot" },
	{ 0xe2c17b5d, "__SCT__might_resched" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x1000e51, "schedule" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x92540fbf, "finish_wait" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0x69acdf38, "memcpy" },
	{ 0x6ad2b3e, "module_layout" },
};

MODULE_INFO(depends, "usbserial");

MODULE_ALIAS("usb:v1A86p7523d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86p5523d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "B5B0F232F54185D83130595");
