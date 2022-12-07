nc localhost "${2:-8765}" | defmt-print -e target/thumbv7em-none-eabihf/debug/"$1"
