#!/bin/sh

currentDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null 2>&1 && pwd)"

if ! command -v 'cargo'; then
	echo 'You need to install Rust onto your computer before starting this tutorial. https://www.rust-lang.org/tools/install'
	exit 1
fi

rustup default nightly
rustup target add thumbv7em-none-eabihf

echo "Installing cargo-binutils for rust-objcopy"
if cargo --list | grep -q 'binstall'; then
	echo "Using cargo binstall"
	cargo binstall cargo-binutils
else
	echo "Using cargo install"
	cargo install cargo-binutils
fi

echo "Adding Teensy Target and llvm-tools-preview"
rustup component add llvm-tools-preview

# Linux Needs libusb-dev
if type apt-get > /dev/null 2>&1; then
	sudo apt-get update && apt-get install -y libusb-dev
elif type brew > /dev/null 2>&1; then
	brew install libusb libusb-compat
fi

# unzip the teensy loader
unzip teensy_loader_cli.zip -d teensy_loader_cli
