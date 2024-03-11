#!/usr/bin/env python3

#
# Converts the bin/nib file into a Rust [u8; X] buffer
#
# Both the fpga and kicker bins must be converted into a buffer
# so we can dynamically program them on robot startup
# This is the easy fix until a file system is built
#

import argparse
import os
import sys

def binary_to_rust(binary: str, output: str):
    # Check the Binary File Exists
    if not os.path.exists(binary):
        print("Binary File Not Found")
        sys.exit(1)
    
    data = None
    with open(binary, "rb") as file:
        data = file.read()
        
    if data is None:
        print(f"Failed to open binary file: {binary}")
    
    num_bytes = len(data)
    print(f"{binary} has {num_bytes} bytes of data.")
    
    buffer_string = ", ".join([hex(d) for d in data])
    file_string = f"pub(crate) const KICKER_BYTES: [u8; {num_bytes}] = [{buffer_string}]"
    
    print("Exporting Buffer")
    with open(output, "w+") as out:
        out.write(file_string)
        
    print("Done")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--binary", help="Binary File Location", required=True)
    parser.add_argument("-o", "--output", help="Output File Location", required=True)
    args = parser.parse_args()
    binary_to_rust(args.binary, args.output, args.buffer_name)
