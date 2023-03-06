# Switchy

Switchy is an attempt at a custom programmable macro keyboard with rust firmware.

The PCB has capacity for:

1. 24x switches,
2. 4x rotary encoder with buttons,
3. 2x two-axis joysticks with push button.

## Development setup

See `template_README.md` for details.

To program the device plug in the STlink and run:

```console
cargo rr
```

> Depending on your target device, change the `runner` section in
> `.cargo/config` to match your device. It defaults to the MCU used in a "black
> pill".

To check binary size for the default binary (`switchy`):

```bash
cargo size --release
```
