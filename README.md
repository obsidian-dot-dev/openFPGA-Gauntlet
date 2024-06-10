# Gauntlet

Gauntlet-compatible FPGA core for Analogue Pocket.

Based on the FPGA Gauntlet core by d18c7db, ported from the [MiSTer version](https://github.com/MiSTer-devel/Arcade-Gauntlet_MiSTer).

## Compatibility

This core supports the arcade games that run on the Atari Gauntlet board.  The list of games includes:

* Gauntlet (both 2 and 4-player versions)
* Gauntlet II (both 2 and 4-player versions)
* Vindicators Part II

## Features

### Service Mode

Toggling "Service mode" in the interact menu will reset the game to its service menu.  From here you can set the difficulty, health-per-coin, and other game parameters, as well as run through the various diagnostics tests.

Toggle "service mode" off to reset the game with the settings applied.  

Note: settings do not persist after exiting the core

### Gauntlet - Character Selection

Exclusive to Gauntlet (original, 4-player cabinet), settings have been added in the interact menu to dynamically map which character is controlled by which joystick.  This allows any player to control any character without having to pair multiple controllers.

### Vindicators Part II - Tank Controls

On Pocket, the tank controls are mapped as follows:

Left Movement -- DPad up/down
Right Movement -- Face buttons X/B
Rotate Turret Left -- DPad right
Rotate Turret Right -- Face button Y
Fire Left -- Trigger L
Fire Right -- Trigger R

When docked and paired with a dual-analogue controller, the game supports twin-stick tank controls as folows:

Left Movement -- Left stick up/down
Right Movement -- Right stick up/down
Rotate Turret Left -- Trigger L1
Rotate Turret Right -- Trigger R1
Fire Left -- Trigger L2
Fire Right -- Trigger R2

## Usage

*No ROM files are included with this release.*  

Install the contents of the release to the root of the SD card.

Place the necessary `.rom` files for the supported games onto the SD card under `Assets/gauntlet/common`.

To generate the `.rom` format binaries used by this core, you must use the MRA files included in this repo, along with the corresponding ROMs from the most recent MAME release.

In order to play Vindicators part 2, you must run [a python script](https://github.com/MiSTer-devel/Arcade-Gauntlet_MiSTer/blob/main/scripts/descramble_2J.py) `descramble_2J.py 136059-1184.2j 136059-1184.des.2j` to descramble the address lines, and then copy the binary output file `136059-1184.des.2j` into the zip file with the original ROMs.

## History

v0.9.0
* Initial Release.

## License

All new materials for this port are licensed under the terms of the GPLv3.

Please see the headers and license files for the licensing terms of the individual dependencies.

## Attribution

```
Original Gauntlet FPGA-compatible core by Alex 
https://github.com/d18c7db/Gauntlet_FPGA
```
