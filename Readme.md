# RC2014 FPGA System

The RC2014 FPGA system is designed to emulate the classic RC2014 computer architecture while providing flexibility for experimentation and future expansion. The system, currently in synthesis, incorporates the following key components:

## Processor
- 1 x Z80 Processor (Fixed): The system features a Z80 processor, offering compatibility with the RC2014 standard.

## Memory
- 512K RAM (Fixed): A fixed 512KB RAM module is integrated into the system, providing sufficient memory for program execution and data storage.
- 512K ROM (Fixed): A fixed 512KB ROM module is included to store the system's firmware and user programs.
- Z2 Memory paging
## Communication Interfaces
- 2 x ACIA 6850B (Internal): Two ACIA 6850B modules are integrated for asynchronous communication. These modules can be disconnected, allowing for flexibility in the configuration.
- 1 x Z80CTC (Internal/Disconnectable): The Z80CTC (Counter/Timer) module is included, providing timing and event generation capabilities. It can be disconnected for configuration flexibility.

## Sound Interface
- 1 x AY-3-8910 (Internal/Disconnectable): The AY-3-8910 sound interface module is integrated, offering programmable sound generation capabilities. It can be disconnected for configurational flexibility.

## OSD Integration
- The system supports the On-Screen Display (OSD) device. There are two RC2014 related subpages. Boot switches and devicesWhen connected, the OSD device is utilized internally. If disconnected, the system automatically searches for the OSD device on the bus.

## Dynamic Configuration
- The architecture allows for the dynamic addition or removal of internal disconnectable devices through the OSD. This feature enables testing and evaluation of real devices to  help the development process.

The RC2014 FPGA system is under active development , combining classic RC2014 components with modern flexibility and configurability. The OSD integration enhances the system's adaptability, allowing users to experiment with various configurations of internal disconnectable devices.

*Note: The system's flexibility to disconnect and dynamically configure internal devices via OSD facilitates real-world testing, enabling synthesis adjustments based on actual device behavior.*
