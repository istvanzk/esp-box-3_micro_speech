# Micro Speech Example for ESP32-S3-BOX-3

![Exp](https://img.shields.io/badge/Dev-Experimental/Fork-orange.svg)
[![Lic](https://img.shields.io/badge/License-Apache2.0-green)](http://www.apache.org/licenses/LICENSE-2.0)
![Py](https://img.shields.io/badge/Python-3.12+-green)
![IDF](https://img.shields.io/badge/ESP--IDF-5.3-green)
![ADF](https://img.shields.io/badge/ESP--ADF-2.7-green)
![HW](https://img.shields.io/badge/HW-ESP32--S3--BOX--3-green)
![Ver](https://img.shields.io/badge/Version-0.1-blue)

**This is a modified micro_speech example from [esp-tflite-micro](https://github.com/espressif/esp-tflite-micro).**

The micro_speech code in this repo uses ESP-ADF audio pipeline with noise supression (NS) and voice activity detection (VAD), that captures dual-mic input and feeds it to micro_speech ML model. The new esp_codec_dev APIs for IDF v>5 and ADF v2.7 pipeline is used. The code here is meant to be a drop-in replacement for the the original code in [esp-tflite-micro/examples/micro_speech](https://github.com/espressif/esp-tflite-micro/tree/master/examples/micro_speech).

The implemented audio pipeline is: 

[ES7210 ADC] → [I2S Stream Reader] → [NS Filter] → [Raw Audio Buffer] → [Downmix to Mono] → [micro_speech Model]

The default micro_spech ML model is a 20 kB model that can recognize 2 keywords, "yes" and "no", from speech data.
The application listens to its surroundings with a microphone and indicates
when it has detected a word by displaying data on a screen.

**NOTE**: The modified micro_speech code in this repo _is being tested_ with ESP-IDF version `release/v5.3` and ESP-ADF version `release/v2.7` 
and only on the [ESP32-S3-BOX-3](https://github.com/espressif/esp-box/blob/master/docs/hardware_overview/esp32_s3_box_3/hardware_overview_for_box_3.md) device.

<details>
<summary>Modified files</summary>

The current code V0.1 succesfully compiles with the follwowing modifications:
* [CMakeLists.txt](CMakeLists.txt)
* [sdkconfig.defaults](./sdkconfig.defaults)
* [main/CMakeLists.txt](./main/CMakeLists.txt)
* [main/idf_component.yml](./main/idf_component.yml)
* [main/main.cc](./main/main.cc)
* [main/audio_provider.cc](./main/audio_provider.cc), [main/audio_provider.h](./main/audio_provider.h)

</details>


## Deploy to ESP32-S3-BOX-3

The following instructions will help you build and deploy this sample
to [ESP32](https://www.espressif.com/en/products/hardware/esp32/overview)
devices using the [ESP IDF](https://github.com/espressif/esp-idf).


### Install the ESP IDF

Follow the instructions of the
[ESP-IDF get started guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html)
to setup the toolchain and the ESP-IDF itself.
[IDF v5.3 is required](https://github.com/espressif/esp-adf/tree/release/v2.x) when using ESP-ADF v2.7.

The next steps assume that the
[IDF environment variables are set](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html#step-4-set-up-the-environment-variables) :

 * The `$IDF_PATH` environment variable is set
 * `idf.py` and Xtensa-esp32 tools (e.g. `xtensa-esp32-elf-gcc`) are in `$PATH`

### Install the ESP ADF

Follow the official [installation guide](https://docs.espressif.com/projects/esp-adf/en/latest/get-started/index.html).

```
git clone -b release/v2.7 --recursive https://github.com/espressif/esp-adf.git
cd asp-idf
./install.sh
. ./export.sh
```
The next steps assume that:
 *  The `$ADF_PATH` environment variable is set
 * **IMPORTANT**: A component file in the ESP-ADF source needs to be modified to avoid dependency conflict with the [esp-box](https://github.com/espressif/esp-box/tree/master) component. 
 Edit the `esp-adf/components/audio_board/idf-component.yml` to contain:
  ```
  dependencies:
    esp_lcd_ili9341: "^2.0.1"
  ```

### Install esp-tflite-micro

Clone the official [esp-tflite-micro](https://github.com/espressif/esp-tflite-micro) repo.
Replace (drop-in) the code in `esp-tflite-micro/examples/micro_speech` with the code from this repo.


### Building the example


```
cd esp-tflite-micro/examples/micro_speech
idf.py fullclean
idf.py build
```

<details>
<summary>Some parts of the expected, succesful, compile output</summary>

```
...

NOTICE: Processing 18 dependencies:
NOTICE: [1/18] espressif/button (4.1.3)
NOTICE: [2/18] espressif/cmake_utilities (0.5.3)
NOTICE: [3/18] espressif/esp-box-3 (3.0.2)
NOTICE: [4/18] espressif/esp-dsp (1.6.0)
NOTICE: [5/18] espressif/esp-nn (1.1.2)
NOTICE: [6/18] espressif/esp-tflite-micro (*) (.../espsdk/esp-tflite-micro)
NOTICE: [7/18] espressif/esp_lcd_ili9341 (2.0.1)
NOTICE: [8/18] espressif/esp_lcd_touch (1.1.2)
NOTICE: [9/18] espressif/esp_lcd_touch_gt911 (1.1.3)
NOTICE: [10/18] espressif/esp_lcd_touch_tt21100 (1.1.1)
NOTICE: [11/18] espressif/esp_lvgl_port (2.6.2)
NOTICE: [12/18] espressif/esp_websocket_client (1.5.0)
NOTICE: [13/18] espressif/icm42670 (2.0.2)
NOTICE: [14/18] espressif/jsmn (1.1.0)
NOTICE: [15/18] espressif/nghttp (1.65.0)
NOTICE: [16/18] espressif/zlib (1.3.1)
NOTICE: [17/18] lvgl/lvgl (9.3.0)
NOTICE: [18/18] idf (5.3.4)
...

<< lots of warnings, from various components and libraries (deprecated uses and shadowed declarations) >>
...

Bootloader binary size 0x59f0 bytes. 0x2610 bytes (30%) free.
[2416/2417] Generating binary image from built executable
esptool.py v4.10.0
Creating esp32s3 image...
Merged 2 ELF sections
Successfully created esp32s3 image.
...

micro_speech.bin binary size 0x829c0 bytes. Smallest app partition is 0x100000 bytes. 0x7d640 bytes (49%) free.

Project build complete. To flash, run:
 idf.py flash
or
 idf.py -p PORT flash
or
 python -m esptool --chip esp32s3 -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 2MB --flash_freq 80m 0x0 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/micro_speech.bin
or from the ".../esp-tflite-micro/examples/micro_speech/build" directory
 python -m esptool --chip esp32s3 -b 460800 --before default_reset --after hard_reset write_flash "@flash_args"

```
</details>



### Load and run the example

To flash (replace `/dev/ttyUSB0` with the device serial port):
```
idf.py --port /dev/ttyUSB0 flash
```

Monitor the serial output:
```
idf.py --port /dev/ttyUSB0 monitor
```

Use `Ctrl+]` to exit.

The previous two commands can be combined:
```
idf.py --port /dev/ttyUSB0 flash monitor
```

### Sample output

  * When a keyword is detected you will see following output sample output on the log screen:

```
Heard yes (<score>) at <time>
```
