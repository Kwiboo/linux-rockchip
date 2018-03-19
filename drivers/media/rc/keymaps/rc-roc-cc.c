/* Keytable for ROC-RK3328-CC IR Remote Controller
 *
 * Copyright (c) 2017 ROC-RK3328-CC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <media/rc-map.h>
#include <linux/module.h>

static struct rc_map_table roc_cc[] = {
	{ 0x28d7, KEY_POWER },
	{ 0xc837, KEY_MUTE },
	{ 0xe01f, KEY_ENTER},
	{ 0xc03f, KEY_UP },
	{ 0x40bf, KEY_DOWN },
	{ 0x708f, KEY_LEFT },
	{ 0x58a7, KEY_RIGHT },
	{ 0x1ae5, KEY_VOLUMEDOWN },
	{ 0xd02f, KEY_VOLUMEUP },
	{ 0x3ac5, KEY_WWW },
	{ 0x807f, KEY_BACK },
	{ 0x12ed, KEY_HOME },
};

static struct rc_map_list roc_cc_map = {
	.map = {
		.scan    = roc_cc,
		.size    = ARRAY_SIZE(roc_cc),
		.rc_type = RC_TYPE_NEC,
		.name    = RC_MAP_ROC_CC,
	}
};

static int __init init_rc_map_roc_cc(void)
{
	return rc_map_register(&roc_cc_map);
}

static void __exit exit_rc_map_roc_cc(void)
{
	rc_map_unregister(&roc_cc_map);
}

module_init(init_rc_map_roc_cc)
module_exit(exit_rc_map_roc_cc)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ROC-RK3328-CC");
