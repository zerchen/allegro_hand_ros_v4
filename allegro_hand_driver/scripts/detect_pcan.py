#! /usr/bin/python

# Automatically detects the /dev/pcanusbN file for controlling the CAN device.
#
# Searches for all possible matches and uses regexp to look for a file of the
# form /dev/pcanusbN. If exactly one is found, prints it and returns it (so this
# is usable within another script). Otherwise, returns the empty string.
#
# Author: Felix Duvallet <felix.duvallet@epfl.ch>

import glob
import re
import sys

def pcan_search(interface="usb"):
    # Look for all possible file matches.
    filelist = glob.glob(f'/dev/pcan{interface}*')

    # Search for the correct format: pcanusbN
    device_re = re.compile(f'.*pcan{interface}[0-9]+')

    gen = (x for x in filelist if device_re.search(x))
    valid_files = [x for x in gen]

    return valid_files

if __name__ == '__main__':
    search_list = ['usb', 'pci']
    available_list = []

    for iface in search_list:
        available_list += pcan_search(iface)

    if(not available_list):
        print(f"No pcan device found.", file=sys.stderr)
    if(len(available_list) > 0):
        if(len(available_list) > 1):
            print(f"More than one pcan device available, taking the first one of: {available_list}", file=sys.stderr)
        print(available_list[0])