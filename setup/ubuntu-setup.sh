#!/usr/bin/bash

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