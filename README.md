# Mega Blaster 2

Demo video can be found here: https://www.youtube.com/watch?v=4sKwGWfikdc

[Instruction Manual](https://github.com/AidanHockey5/MegaBlaster2/raw/master/man/MegaBlaster2InstructionManual.pdf)

[Store Link](https://www.aidanlawrence.com/product/mega-blaster-2/)

[Discord Link](https://discord.gg/M2skqkZhw2)

This project contains the source material for the Arcade Classic 2, a hybrid hardware/software YM2151 video game music (VGM) player. This device is based off of the [Mega Blaster 2 platform](https://github.com/AidanHockey5/MegaBlaster2) which was designed to play Sega Genesis/Megadrive VGM files.
This project utilizes a hybrid approach for rendering audio. All FM portions of the VGM file are played through a genuine YM2151 sound chip, while any extra soundchips, namely PCM soundchips, are pre-rendered and fed into an audio mixer circuit.
While it would be awesome to have genuine hardware support for every chip that was ever paired up with the YM2151, there are several factors that make that approach completely unfeasible. Namely, the vast amount of hardware configurations, memory requirements, sheer board space required, and rarity of components.
Therefore, this player is essentially playing back a WAV file for the external chips while also driving the genuine FM sound generator and mixing them together.
The result sounds pretty awesome!

# Special VGM File Requirements

The Arcade Classic 2 will read standard VGM/VGZ files just fine, however, in order to enable pre-rendered PCM support, you will need to quickly run your VGM files through a tool I've created. This tool will render out any portions of the VGM file that aren't controlling the YM2151 into a WAV file. The tool will then append the WAV data to the original VGM file. Finally, the tool will change the VGM header information so that the Arcade Classic 2 will recognize it as a PCM-compatible VGM file.

This tool is only compatible with Windows systems for now. It's fairly simple to use:
[Download the VGM_To_VGP Tool Here!!!](https://aidanlawrence.com/tools/ee/arcadeclassic2/vgm_to_vgp.zip)
1) Drag-and-drop VGM/VGZ files or folders containing VGM/VGZ onto the executable
2) Let the program render out each track
3) Add the newly created "VGP" files to your SD card!

Please note: WAV data is very bulky and will take up significantly more space than simple VGM/VGZ files.
This tool was written in C++ with Visual Studio 2017. It invokes [VGMPlay](https://github.com/vgmrips/vgmplay) to render the soundchip WAV data, then invokes [SoX](http://sox.sourceforge.net/) to ensure that the WAV data is in the correct format.
[The source code for VGM_To_VGP can be found here](https://github.com/AidanHockey5/YM2151_Arcade_Classic_2/tree/master/tools/vgmtovgp)

# VGM Files

VGM files are logged datasets of the data that would have been sent to the real sound chips of whatever console or arcade machine was being sampled. The Arcade Classic 2 simply reads-in this data and sends it to the sound chips at the exact rate specified in the VGM spec, accurately recreating the original sound.

You can find lots of VGM files here for all sorts of systems: https://vgmrips.net/packs/
The Arcade Classic 2 is only designed for soundtracks featuring the YM2151 sound chip. Technically, you could pre-render any VGM track you wanted and play it just fine, but that would essentially just be a WAV player with extra steps.

Please make sure to remove any non-vgm/vgz files before adding them to your Arcade Classic 2's SD card, such as album art images, playlist files, and text files.

# VGM vs. VGZ

VGZ files are gzip compressed versions of VGM files. The Arcade Classic 2 is capable of automatically decompressing these vgz files on-the-fly at a slight performance penalty. If you'd like faster load times for each track, consider decompressing the VGZ files in advance by opening them with a program like [7zip](https://www.7-zip.org/)
The Arcade Classic 2 does not require file extensions in order to operate as it will verify files based on their internal headers.

# Building Your Own Unit

Here are some quick tips for building your own board.

1) You can find an interactive Bill of Materials (BOM) within the `schematic->bom` folder. Open the HTML file in any browser for an easy-to-use part position guide.
2) I use [KiCAD](https://kicad.org/) as my EDA suite.
4) I buy most of my components from [LCSC](https://lcsc.com/). I buy my PCBs from [JLCPCB](https://jlcpcb.com/). If you buy your PCBs from JLCPCB, you can often get a shipping discount on your LCSC parts order.
5) You will need an [Atmel ICE](https://www.microchip.com/DevelopmentTools/ProductDetails/ATATMEL-ICE) or [Jlink](https://www.segger.com/products/debug-probes/j-link/) to flash the bootloader. Once you have flashed the bootloader, all subsequent programming can be done via the on-board USB port. A cheaper option would be to grab a [Jlink EDU Mini](https://shop-us.segger.com/J_Link_EDU_mini_p/8.08.91.htm) as they feature the full functionality of a professional Jlink Unit with the stipulation that it is ONLY to be used for educational purposes. I personally use an Atmel ICE unit. You can find the bootloader bin file in the `bootloader` folder of the repository. [You can find an example of me programming the bootloader using Microchip Studio here.](https://youtu.be/FjPftGuLnGg?t=9259)
6) Once you've flashed the bootloader, you can drag the [.uf2 firmware file found on the releases page](https://github.com/AidanHockey5/MegaBlaster2/releases) on to the "drive" that pops up on your computer, or build from source. If you can't see any "drive," make sure the bootloader mentioned above is flashed correctly, then try double-tapping the RESET button. 
7) If your OS reports a USB fault, you may have a short somewhere on your USB IO lines, power lines, or your 32.768KHz crystal is not hooked up correctly.
8) I use [Visual Studio Code](https://code.visualstudio.com/) with the [PlatformIO](https://platformio.org/install/ide?install=vscode) extension.
9) Make sure your SN76489 PSG chip is orientated correctly. On this board, the PSG is upside-down relative to the rest of the board. The Flash IC is also upside-down. This was done for routing purposes. A small star on the silk-screen indicates the pin-1 position of every IC.
10) For general usage instructions, [please refer to the manual](https://github.com/AidanHockey5/MegaBlaster2/raw/master/man/MegaBlaster2InstructionManual.pdf).

# Reporting Bugs

As you can imagine, this is quite a complex project. There are bound to be bugs to squish, so if you happen to find any, [please report them over on the issues page](https://github.com/AidanHockey5/YM2151_Arcade_Classic_2/issues) so I can tkae a look at them! Please try to document the steps it takes to recreate the bug if you happen to know them.

# Schematic

![Schematic](https://github.com/AidanHockey5/YM2151_Arcade_Classic_2/raw/master/schematic/ArcadeClassic2.png)

# License
The source code to this project is licensed under the AGPL 3.0 license.

The printed circuit board design files for the Arcade Classic 2, including it’s schematic, gerber files, KiCAD files, bill-of-materials, and 3D models are licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 license.

You are permitted to build your own Arcade Classic 2 units for personal use. You are not permitted to sell said units.
