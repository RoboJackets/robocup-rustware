#!/bin/bash

if [[ -z $2 ]]; then
    cargo run --release --example $1 --target thumbv7em-none-eabihf
else
    cargo run --release --example $1 --features $2 --target thumbv7em-none-eabihf
fi

sleep 0.5

# sudo minicom
# sudo screen /dev/tty.usbmodem14101