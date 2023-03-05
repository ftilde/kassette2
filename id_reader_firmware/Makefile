PROJECT_NAME=id_reader_firmware
ELF=target/thumbv6m-none-eabi/release/${PROJECT_NAME}
ELF2UF2=${HOME}/.cargo/bin/elf2uf2-rs #install with `cargo install --locked elf2uf2-rs
TTY=/dev/ttyACM0
BAUDRATE=8000

all: ${ELF}

${ELF}: src/*.rs ../config/src/*.rs
	cargo build --release

.PHONY: flash tty

flash: ${ELF}
	${ELF2UF2} -d ${ELF}

tty:
	for i in $$(seq 10); do com ${TTY} ${BAUDRATE} && break; sleep 1; done
