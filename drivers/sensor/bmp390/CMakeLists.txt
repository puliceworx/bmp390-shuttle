#
# Copyright (c) 2020 Facebook, Inc. and its affiliates
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()

zephyr_library_sources(bmp3.c)
zephyr_library_sources(bmp390.c)
zephyr_library_sources_ifdef(CONFIG_BMP390_TRIGGER bmp390_trigger.c)
