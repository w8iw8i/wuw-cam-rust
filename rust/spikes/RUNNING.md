# Running the spikes on a camera

Each spike replaces the camera's firmware until you flash the C++ firmware back.
Saved settings (WiFi, password, stream mode, BWO name) live in the `nvs`
partition, which none of these steps touch.

## Once per machine

    cargo install espup espflash
    espup install
    source ~/export-esp.sh

## Before the first flash: back up the whole chip

    espflash board-info
    espflash read-flash 0 0x1000000 ~/wuwcam-<cam>-backup-<date>.bin

## Spike C: WiFi AP + HTTP

    cd rust/spikes/spike-net && cargo run --release

Join WiFi `wuw-rust` (password `wuwuwuwu`), then from the repo root:

    python3 tools/spike_net_stress.py --secs 600

PASS needs zero errors and no heap loss.

## Spike B: SD card (read-only)

    cd rust/spikes/spike-sd && cargo run --release

Look for `SPIKE B READ: ... KB/s` and `SPIKE B: PASS`. The write test
(`--features destructive-write`) overwrites 10 MiB of the card: spare, empty
card only.

## Spike A: camera

    cd rust/spikes/spike-camera && cargo run --release

Look for `SPIKE A ID: 0x5640 PASS`, then `SPIKE A: <good> good / <bad> bad ...
fps` and `SPIKE A: PASS`. If the log says the DMA ring overran, note it.

## Back to the C++ firmware

    pio run -e wuwcam -t upload

Check the camera boots and joins its saved WiFi. Write every number into
`docs/rust-spike-results.md`.
