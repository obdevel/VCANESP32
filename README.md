<img align="right" src="ArduinoVLCB.png"  width="150" height="75">

# VCAN2040
This library provides a software CAN interface on a Raspberry Pi Pico when used in conjunction with the VLCB_Arduino library
suite.  The main VLCB-Arduino library can be found at [VLCB](https://github.com/SvenRosvall/VLCB-Arduino) 

This VLCB library code is based on a [CBUS library](https://github.com/MERG-DEV/CBUSACAN2040) created by Duncan Greenwood
and extended by members of [MERG](https://www.merg.org.uk/). See below under Credits.
This code has been adapted for use with VLCB.

See [Design documents](https://github.com/SvenRosvall/VLCB-Arduino/blob/main/docs/Design.md) for how this library is structured.

## Examples
There are two versions of the example that, from the users perspective, are functionally idential.
The first uses a single core in the Pico and is identified as VLCB_4in4out_Pico_s, where 's' stands
for single core.  The other core will be dormant in a low power state.
The second version makes use of both cores in the processer and is identifed as VLCB_4in4out_Pico_d,
where 'd' stands for dual core.  This is organised such that the VLCB library runs in core 0 and the
application runs in core 1.

The docs folder in this repository provides notes on how to use this library with the 4in4out examples.
It also holds a circuit schematic suitable for the code as written.

## Dependencies
See dependancies in VLCB4in4out_README.md in this repository.

## Hardware

Currently supports the Raspberry Pi Pico using Earle Philhowers Arduino IDE board support
[Pico Arduino IDE](https://github.com/earlephilhower/arduino-pico) Full instructions on how to do this in the associated
[documentation](https://arduino-pico.readthedocs.io/en/latest/)

## Getting help and support

At the moment this library is still in development and thus not fully supported.

If you have any questions or suggestions please contact the library maintainers
by email to [martindc.merg@gmail.com](mailto:martindc.merg@gmail.com) or create an issue in GitHub.

## Credits

* Duncan Greenwood - Created the CBUS library for Arduinos upon which this VLCB library is based on.
* Sven Rosvall - Converted the CBUS library into the main VLCB-Arduino library suite.
* Martin Da Costa - Contributed to Sven's work and converted the CBUSACAN2040 library to this VCAN2040.

## License

The code contained in this repository and the executable distributions are licensed under the terms of the
[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License](LICENSE.md).

If you have questions about licensing this library please contact [martindc.merg@gmail.com](mailto:martindc.merg@gmail.com)
