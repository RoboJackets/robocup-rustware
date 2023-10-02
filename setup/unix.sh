#!/bin/sh

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

currentDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null 2>&1 && pwd)"

echo "Cloning Teensy Loader CLI GitHub Repository"
git clone https://github.com/PaulStoffregen/teensy_loader_cli.git "${currentDir}/teensy_loader_cli"

echo "${currentDir}"

echo "Making Teensy Loader CLI"
make -C "${currentDir}/teensy_loader_cli"

echo "Moving Executable"
mv "${currentDir}/teensy_loader_cli/teensy_loader_cli" ${currentDir}/../teensy_loader_cli

echo "Cleaning Up"
rm -rf "${currentDir}/teensy_loader_cli"