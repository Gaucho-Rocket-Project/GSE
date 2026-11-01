# NUCLEO-H753ZI Rust ping/pong test

A bare-metal Rust firmware and a Rust terminal responder. They use the onboard ST-LINK virtual COM port: USART3 TX=PD8, RX=PD9, 115200 baud, 8 data bits, no parity, 1 stop bit, no flow control. Plug the computer into the **ST-LINK USB connector**, not the target USB connector. No additional wires are normally needed with the board's default VCP solder-bridge configuration. See [board documentation](https://docs.zephyrproject.org/latest/boards/st/nucleo_h753zi/doc/index.html) and [ST's UM2407](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf).

## Protocol

ASCII lines, terminated by LF (CRLF is accepted):

    board -> host: PING 0\n
    host -> board: PONG 1\n
    board -> host: PING 1\n
    host -> board: PONG 2\n

Both counters start at 0 on process/board reset. When a ping matches the host's current accepted count, the host increments it modulo 2^32 and sends the new count. When it doesn't match (including a retry after a lost pong), the host returns its current count without incrementing. The board adopts the count in any valid pong, even if different from its own, then sends the next ping after 250 ms. If no valid pong arrives within 5 seconds, it retransmits the same ping; malformed/oversized lines are ignored. The host's counter is authoritative and only stored in RAM. For example, after restarting the host, the board will resync to 0. This simple test has no checksums, sequence IDs, or persistent state, and is not a production reliable messaging protocol; delayed or duplicated valid pongs cannot be distinguished from current replies. Counter wrap is modulo 2^32.

## Flash the included binary

The `artifacts/` directory contains a prebuilt ELF and raw binary. Install [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) and connect the ST-LINK USB cable. From this directory, using its CLI on your PATH:

```sh
STM32_Programmer_CLI -c port=SWD -w artifacts/nucleo-ping.bin 0x08000000 -v -rst
```

On some systems the executable has a different path; use the full path to it. This writes target flash. Make sure the board is the NUCLEO-H753ZI, not a different MCU. After flashing, the board starts transmitting immediately; if the terminal is not running yet, it retries every 5 seconds.

## Run the host

Install [Rust](https://rustup.rs/). Identify the ST-LINK serial port (commonly `/dev/ttyACM0` on Linux, `/dev/cu.usbmodem...` on macOS, or `COM3` on Windows), then:

```sh
cd host
cargo run --release -- /dev/ttyACM0
```

Replace the port path. Only one program can open the port at a time. On Linux, grant your user access to the serial port (often via the `dialout` group and a new login); on Windows you may need the ST VCP driver. Stop with Ctrl-C. Reset the board if you want to start its counter again at 0; the host resynchronizes it to its own count.

## Rebuild firmware

```sh
rustup target add thumbv7em-none-eabihf
cd firmware
cargo build --release
```

The output is `target/thumbv7em-none-eabihf/release/nucleo-ping` (ELF). To make a raw binary:

```sh
rustup component add llvm-tools
cargo install cargo-binutils
cargo objcopy --release -- -O binary nucleo-ping.bin
```

Flash the generated binary as above, or use a debugger supporting ELF files. The firmware uses reset-default 64 MHz HSI clocks and configures USART3 directly. It assumes a normal hardware reset, no prior bootloader clock configuration, and the board's default ST-LINK/USART routing. It uses SysTick for the retry interval, which is approximate because HSI is not precision calibrated.

## Test status

Host unit tests: `cd host && cargo test`. The included firmware was cross-compiled successfully. Hardware flashing and end-to-end serial operation were not tested here because no physical board was available.
