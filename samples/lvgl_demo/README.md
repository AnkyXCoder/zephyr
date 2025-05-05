# LVGL Demo

## Overview

A sample showcasing upstream LVGL demos.

1. Music
   The music player demo shows what kind of modern, smartphone-like user interfaces can be created on LVGL.

2. Benchmark
   The benchmark demo tests the performance in various cases. For example rectangle, border, shadow, text, image blending, image transformation, blending modes, etc.

3. Stress
   A stress test for LVGL. It contains a lot of object creation, deletion, animations, styles usage, and so on. It can be used if there is any memory corruption during heavy usage or any memory leaks.

4. Widgets
   Shows how the widgets look like out of the box using the built-in material theme.

## Requirements

### Hardware Requirements

* One of the Suported Boards
* [1.28inch Touch LCD Breakout Board](https://www.waveshare.com/1.28inch-Touch-LCD.htm)

Note that other input devices types are not demonstrated in these demos, namely keyboards, keypads (`zephyr,lvgl-keypad-input`), rotary encoders (`zephyr,lvgl-encoder-input`) and hardware buttons (`zephyr,lvgl-button-input`).

## Supported Boards

* `esp32s3_touch_lcd/esp32s3/procpu`

## Building and Running

Building this sample using `west`:

```shell
    west build -p -b <board_name>
```

## References

More details can be found in [LVGL demos Readme](https://github.com/zephyrproject-rtos/lvgl/blob/zephyr/demos/README.md).
