## 2024-02-21 Design Review Notes

* The ESP32 symbol on the schematic has no pin numbers!
  Please add them, else there are no connections to the device.
* I think the footprint for the ESP32 isn't quite right.
  Doesn't it have two rows of 18 pins?
* Don't put footprints or schematic symbols in the design directory.
  They need to be in a library.  Read about making libraries in the
  documentation, please.
* It's _very_ helpful to have datasheet links on all the symbols.
  Then, you can just hit 'd' while pointing at a symbol and get the datasheet.
