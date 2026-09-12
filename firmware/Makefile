TARGET = target/thumbv6m-none-eabi

dev:
	DEFMT_LOG=info cargo embed defmt
	rust-size $(TARGET)/debug/c92-firmware

connect:
	cargo embed connect

release:
	DEFMT_LOG=off cargo embed --release
	rust-size $(TARGET)/release/c92-firmware