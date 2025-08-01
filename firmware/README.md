# cuppa firmware

This project uses the Pi PICO extension in VSCode to build and flash the Pi PICO chip inside of CUPPA. To build:

```
git clone https://github.com/MZandtheRaspberryPi/cuppa
```

Navigate to VSCode's PICO extension and open the folder `cuppa/firmware` as a project. From here you should see the compile button in the bottom right of VSCode. Hitting compile should bring up the terminal, and upon completion without errors you will see a folder `build` and inside of there a file `cuppa.uf2`. If you hold the BOOTSEL button on cuppa and then plug it into the computer, it will come up as a flashdrive. Copy the `.uf2` file onto the flashdrive. Cuppa will restart.

# contributing

We have a `.clang-format` file in the folder that specifies how to format C++ code. Please set your IDE to use this automatically on save.

We also have some unit tests in the `tests` subfolder. Please ensure these pass post-changes. To run these:
```
cd tests
cmake -S . -B build
cmake --build build
cd build && ctest
```