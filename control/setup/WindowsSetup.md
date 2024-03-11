Install Dependencies - run each of these commands in Windows Powershell one after the other to download all necessary dependencies.
 ```sh
 rustup target add thumbv7em-none-eabihf
 ```
 ```sh
 $ cargo install cargo-binutils
 ```
 ```sh
 $ rustup component add llvm-tools-preview
 ```
 - Download teensy loader for windows from here: https://www.pjrc.com/teensy/loader_win10.html. Download both the files from this website and follow instructions on website. You will need to borrow a Teensy to ensure its installed correctly
 - Clone Kelvin's test file
 ```sh
 git clone https://github.com/celcius-plus-273/teensy4-rust-template.git
 ```
 - Type this into command line to rename the teensy.exe to work with the runner
 ```sh
 mv teensy.exe teensy_loader_cli.exe
 ```
 - Then put teensy.exe into teensy4-rust-template folder and run
 ```sh
 cargo run --release --target thumbv7em-none-eabihf
 ```