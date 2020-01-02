#!/bin/sh
SYMLINK="ttyGPRS"
[ -e /dev/${SYMLINK} ] || power_mode gprs on
