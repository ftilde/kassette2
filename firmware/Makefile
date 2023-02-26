PROJECT_NAME=kassette2
ELF=target/thumbv6m-none-eabi/release/${PROJECT_NAME}
ELF2UF2=${HOME}/.cargo/bin/elf2uf2-rs #install with `cargo install --locked elf2uf2-rs

all: ${ELF}

${ELF}: src/*.rs src/*.pio ../config/src/*.rs
	cargo build --release

.PHONY: flash

flash: ${ELF}
	${ELF2UF2} -d ${ELF}
