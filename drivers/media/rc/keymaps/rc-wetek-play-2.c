/* Keytable for WeTek Play 2 Remote Controller
 *
 * Copyright (c) 2017 WeTek
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <media/rc-map.h>
#include <linux/module.h>

static struct rc_map_table wetek_play_2[] = {
	{ 0x5e5f02, KEY_POWER },
	{ 0x5e5f46, KEY_POWER2 },
	{ 0x5e5f10, KEY_MUTE },
	{ 0x5e5f22, KEY_NUMERIC_1 },
	{ 0x5e5f23, KEY_NUMERIC_2 },
	{ 0x5e5f24, KEY_NUMERIC_3 },
	{ 0x5e5f25, KEY_NUMERIC_4 },
	{ 0x5e5f26, KEY_NUMERIC_5 },
	{ 0x5e5f27, KEY_NUMERIC_6 },
	{ 0x5e5f28, KEY_NUMERIC_7 },
	{ 0x5e5f29, KEY_NUMERIC_8 },
	{ 0x5e5f30, KEY_NUMERIC_9 },
	{ 0x5e5f71, KEY_BACKSPACE },
	{ 0x5e5f21, KEY_NUMERIC_0 },
	{ 0x5e5f72, KEY_CAPSLOCK },
	{ 0x5e5f03, KEY_HOME },
	{ 0x5e5f48, KEY_MENU },
	{ 0x5e5f61, KEY_BACK },
	{ 0x5e5f83, KEY_INFO },
	{ 0x5e5f84, KEY_COMPOSE },
	{ 0x5e5f77, KEY_HELP },
	{ 0x5e5f50, KEY_UP },
	{ 0x5e5f4b, KEY_DOWN },
	{ 0x5e5f4c, KEY_LEFT },
	{ 0x5e5f4d, KEY_RIGHT },
	{ 0x5e5f47, KEY_OK },
	{ 0x5e5f44, KEY_VOLUMEUP },
	{ 0x5e5f43, KEY_VOLUMEDOWN },
	{ 0x5e5f41, KEY_CHANNELUP },
	{ 0x5e5f42, KEY_CHANNELDOWN },
	{ 0x5e5f4f, KEY_ZENKAKUHANKAKU },
	{ 0x5e5f82, KEY_TEXT },
	{ 0x5e5f73, KEY_RED },
	{ 0x5e5f74, KEY_GREEN },
	{ 0x5e5f75, KEY_YELLOW },
	{ 0x5e5f76, KEY_BLUE },
	{ 0x5e5f67, KEY_PREVIOUS },
	{ 0x5e5f79, KEY_REWIND },
	{ 0x5e5f80, KEY_FASTFORWARD },
	{ 0x5e5f81, KEY_NEXT },
	{ 0x5e5f04, KEY_RECORD },
	{ 0x5e5f2c, KEY_PLAYPAUSE },
	{ 0x5e5f2b, KEY_STOP },
};

static struct rc_map_list wetek_play_2_map = {
	.map = {
		.scan    = wetek_play_2,
		.size    = ARRAY_SIZE(wetek_play_2),
		.rc_type = RC_TYPE_NEC,
		.name    = RC_MAP_WETEK_PLAY_2,
	}
};

static int __init init_rc_map_wetek_play_2(void)
{
	return rc_map_register(&wetek_play_2_map);
}

static void __exit exit_rc_map_wetek_play_2(void)
{
	rc_map_unregister(&wetek_play_2_map);
}

module_init(init_rc_map_wetek_play_2)
module_exit(exit_rc_map_wetek_play_2)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("WeTek");
