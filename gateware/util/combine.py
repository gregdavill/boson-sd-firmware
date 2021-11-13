#!/usr/bin/env python3

import argparse
import os
import struct
import binascii


def CombineBinaryFiles(flash_regions_final, output_file):
    flash_regions = {}

    offset = None
    for _, base in flash_regions_final.items():
        if offset is None:
            offset = base
        else:
            offset = min(offset, base)

    for filename, base in flash_regions_final.items():
        if base == offset:
            base = base + 8
        new_address = base - offset
        print(f'Moving {filename} from 0x{base:08x} to 0x{new_address:08x}')
        flash_regions[filename] = new_address


    total_len = 0


    with open(output_file, "wb") as f:

        for filename, base in flash_regions.items():
            data = open(filename, "rb").read()
            
            print(f' ')
            print(f'Inserting {filename:60}')
            print(f'  Start address: 0x{base + offset:08x}')
            
            print(f'  Length       : 0x{len(data):08x} bytes')
            print(f'  Gap          : 0x{base - total_len:08x} bytes')
            print(f'           data: ' + f' '.join(f'{i:02x}' for i in data[:16]))
            f.seek(base)
            f.write(data)

            total_len += len(data)

    crc = 0
    magic = 0xb5d930e9

    with open(output_file, "r+b") as f:
        f.seek(8,0)
        crc = binascii.crc32(f.read())

        print(f'Inserting magic=0x{magic :08x} & crc=0x{crc :08x} ')        
        f.seek(0, 0)
        f.write(struct.pack("II", magic, crc))
        
        

