# harry kier box

* use vs code + platformio, android studio sucks, has no dependency management etc
* different programs are in same folder, use build_src_filter in platformio.ini
* use platformio tasks left panel to select which to build or so or better terminal, see below

# development

use plartformio with vs code, don't forget to switch to the correct environment at the bottom!

## build & upload

open platformio terminal, teensy connected via usb:

    # close platformio serial console before!!!
    pio run -e firsttest -t upload
