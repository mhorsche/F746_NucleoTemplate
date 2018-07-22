# STM32 Nucleo-F746ZG

VS Code template project for NUCLEO-F746ZG. This template uses the GNU Arm Embedded Toolchain as well as the OpenOCD server for programming and debugging.

## Requirements

To setup VS Code to build, flash and debug STM32 controllers, it is necessary to ensure the following programs are installed:

* ARM GCC Toolchain: C, C++ and Assembly programming targeting Arm Cortex-M and Cortex-R family of processors

    https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
    
    Use the default installation path (`C:\Program Files (x86)\GNU Tools Arm Embedded`), otherwise you have to adjust the `includePath` in the `.vscode/c_cpp_properties.json`. For simplicity add `C:\Program Files (x86)\GNU Tools Arm Embedded\7 2018-q2-update\bin` to your users `PATH` variable and check if `arm-none-eabi-gcc -v` is available in your terminal:

    ```c
    $ arm-none-eabi-gcc -v
    Using built-in specs.
    COLLECT_GCC=C:\Program Files (x86)\GNU Tools Arm Embedded\7 2018-q2-update\bin\arm-none-eabi-gcc.exe
    COLLECT_LTO_WRAPPER=c:/program\ files\ (x86)/gnu\ tools\ arm\ embedded/7\ 2018-q2-update/bin/../lib/gcc/arm-none-eabi/7.3.1/lto-wrapper.exe
    Target: arm-none-eabi
    Configured with: ...
    Thread model: single
    gcc version 7.3.1 20180622 (release) [ARM/embedded-7-branch revision 261907] (GNU Tools for Arm Embedded Processors 7-2018-q2-update)
    ```

* GNU MCU Eclipse Windows Build Tools: 

    https://gnu-mcu-eclipse.github.io/windows-build-tools/
    
    As mentioned on the install page, it is recommended to use `%userprofile%\AppData\Roaming\GNU MCU Eclipse` for installation. Despite it is not recommended, for simplicity I added `%userprofile%\AppData\Roaming\GNU MCU Eclipse\Build Tools\2.11-20180428-1604\bin` (your build number might vary depending on the version you have installed) to your users `PATH` variable and check if `make --v` is available in your terminal:

    ```c
    $ make --v
    GNU Make 4.2.1
    Built for x86_64-w64-mingw32
    Copyright (C) 1988-2016 Free Software Foundation, Inc.
    License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
    This is free software: you are free to change and redistribute it.
    There is NO WARRANTY, to the extent permitted by law.
    ```

* OpenOCD: GDB Server that can be used with a number of debuggers as the STLink

    http://openocd.org

    The OpenOCD does also provide some `*.cfg` files for different controllers. Again for simplicity add the installation folder to your users `PATH` variable and check if `openocd --v` is available in your terminal:

    ```c
    openocd --v
    GNU MCU Eclipse 64-bits Open On-Chip Debugger 0.10.0+dev-00487-gaf359c18 (2018-05-12-19:30)
    Licensed under GNU GPL v2
    For bug reports, read
            http://openocd.org/doc/doxygen/bugs.html
    ```

Now you should be able to build (`make`) the project and use OpenOCD to flash the attached controller. For debugging you need to install the VS Code plugin:

* Cortex-Debug: Debugging support for ARM Cortex-M Microcontrollers

    https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug

## Usage

As long your installation paths are identical and all above commands run in terminal without providing absolute paths, all predefined settings should be fine. Otherwise check `.vscode/settings.json` and setup your correct paths for the ARM GCC Toolchain and OpenOCD.

### Build & Flash

The normal workflow is to run task `🔨 Build (GCC)` by pressing `STRG+SHIFT+B`. This might take a while and will end with something like:

```c
   text    data     bss     dec     hex filename
   8980     132    1588   10700    29cc build/firmware.elf
arm-none-eabi-objcopy -O ihex build/firmware.elf build/firmware.hex
arm-none-eabi-objcopy -O binary -S build/firmware.elf build/firmware.bin
```

Afterwards run task `🛠️ Program & Verfiy STM (OpenOCD)` by pressing `STRG+P` and type in `task Program` and hit `Enter`. Additional tasks are defined in `.vscode/tasks.json`. Now your microcontroller is flashed and autostarts with the simple blinky project.

### Printf via SWO/SWV

Sadly the SWO/SWV debugging does not work in the VS Code terminal for me, therefore it is necessary to run the ST-Link Utility (https://www.st.com/en/development-tools/stsw-link004.html) and run `ST-LINK > Printf via SWO viewer`. Set the system clock to `216000000` and leave the stimulus port to `0`. Now you should see all printf messages as long you compile the project with `#define DEBUG` (`.vscode/c_cpp_properties.json` global defines in section `defines`).
![Printf via SWO viewe](https://github.com/mhorsche/F746_NucleoTemplate/blob/master/docs/images/STLink_Debug.png)

### Debug in VS Code

To run the Debugger press `F5`. Now you should have access to all debugging features provided by the Cortex-Debug plugin. The `STM32F7x6.svd` provides all register information in the Cortex peripherals section. For additional information see https://marcelball.ca/projects/cortex-debug/ and check `.vscode/launch.json`.
