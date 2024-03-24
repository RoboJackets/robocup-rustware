#!/bin/bash

cargo run --release --example $1 --target thumbv7em-none-eabihf

sleep 0.5

sudo minicom