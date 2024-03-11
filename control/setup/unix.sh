#!/bin/sh

currentDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null 2>&1 && pwd)"

if ! command -v 'cargo'; then
	echo 'You need to install Rust onto your computer before starting this tutorial. https://www.rust-lang.org/tools/install'
	exit 1
fi

echo "Installing cargo-binutils for rust-objcopy"
if cargo --list | grep -q 'binstall'; then
	echo "Using cargo binstall"
	cargo binstall cargo-binutils
else
	echo "Using cargo install"
	cargo install cargo-binutils
fi

echo "Adding Teensy Target and llvm-tools-preview"
rustup target add thumbv7em-none-eabihf
rustup component add llvm-tools-preview

# Linux Needs libusb-dev
if type apt-get > /dev/null 2>&1; then
	sudo apt-get update && apt-get install -y libusb-dev
fi

echo "Cloning Teensy Loader CLI GitHub Repository"
git clone https://github.com/PaulStoffregen/teensy_loader_cli.git "${currentDir}/teensy_loader_cli"

echo "${currentDir}"

echo "Making Teensy Loader CLI"
make -C "${currentDir}/teensy_loader_cli"

echo "Moving Executable"
mv "${currentDir}/teensy_loader_cli/teensy_loader_cli" ${currentDir}/../teensy_loader_cli

echo "Cleaning Up"
rm -rf "${currentDir}/teensy_loader_cli"