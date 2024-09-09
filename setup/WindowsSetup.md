## Rust (and rust dependencies)

I would recommend completing this in wsl, but you should be able to do everything except build the teensy_loader_cli via normal windows.

Go [here](https://www.rust-lang.org/tools/install) and install the RUSTUP-INIT.exe for your respective os (i.e. 32 or 64 bits)

Now we'll need to setup the necessary toolchain configuration for the robocup-rustware repo.

This should install rust to path, but please confirm that running `rustc --version` works before you procede.

First, set the default rust compiler to use the `nightly` compiler (technically this compiler is unstable, but the only feature we use from it is very close to being available in stable rust so it is effectively stable)

```sh
rustup default nightly
```

Next, set the curent target to be `thumb7e-none-eaihf` with:

```sh
rustup target add thumbv7em-none-eabihf
```

To actually run code for the Teensy we will have to copy and transmute the outputted .elf file to a .hex file. Luckily rust's package manager, cargo, has a package that can perform exactly that.

```sh
# LLVM GNU Make alternatives that can modify binary formats
rustup component add llvm-tools-preview
# Cargo bindings for the llvm-tools-preview
cargo install cargo-binutils
```

## Teensy Dependencies

It is a huge pita to actually build the teensy_loader_cli for windows so I stole it from platformio. In the setup folder there should be a zip file named `teensy_loader_cli.zip`. If you extract that folder there should be a teensy_loader_cli.exe file. Move that folder to the base directory of the repo.

Next open `tools/runner.rs` and change the name of the loader on line 25 from `./teensy_loader_cli` to `./teensy_loader_cli.exe`. Now to test if all of the setup was performed correctly run:

```sh
cargo run --example demo --target thumbv7em-none-eabihf
```

Click the program button on the teensy and the onboard led should blink on and off.

## Serial Terminal Setup

Although you can already program the Teensy, it is also necessary to read logs from the teensy so you can actually debug programs. I'm 100% sure theres a ton of ways to do this, but I would probably recommend downloading PuTTY. I think you could technically use the Arduino IDE but thats kind of wack so PuTTY can be downloaded from [here](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html).

Connect the Teensy via usb to the computer and open the Device Manager. Show Hidden USB devices via View > Show hidden devices. Under Ports (COM & LPT) there should be a USB Serial Device on one of the COM ports. Open PuTTY, specify the Connection type of "Serial" and add the COM port to the section under Serial Line. Then, click "Open" and you should be a window displaying the log messages from the Teensy.
