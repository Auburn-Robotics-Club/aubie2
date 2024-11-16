# AUBIE2 Competition Robots

Source code for the software used in AUBIE2's VEXU High Stakes robots.

## About

This is the codebase used by AUBIE2 for our VEXU competition robots. This is a [vexide](https://vexide.dev/) project, written in the Rust programming language. [Check out the documentation](https://vexide.dev/docs/) on the official vexide website for walkthrough-style guides and other helpful learning resources with vexide!

An [API reference](https://docs.rs/vexide) is also provided by docs.rs.

## Development

### Compiling and uploading to a VEX V5 robot

Use the cargo-v5 terminal utility to build and upload this vexide project.

```console
cargo v5 build
```

Use a USB cable to connect to your robot brain or to your controller before using the `upload` subcommand to build and upload the project. Make sure to specify a program slot.

```console
cargo v5 upload
```

### Viewing program output

You can view panic messages and calls to `println!()` using the terminal.
Use a USB cable to connect to your robot brain or controller, then start the terminal:

```console
cargo v5 terminal
```
