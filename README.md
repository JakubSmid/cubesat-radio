# Cubesat Radio (KIN semestral project)
Before you start you need to install few dependencies: libsocketcan, cmake, git, ninja-build.
## nRF52832 DK
We are using nRF52832 DK as the development board. There are two available SDKs for this chip - nRF5 SDK which is in maintenance mode and it is not recommended to use it. The second SDK is called nRF Connect SDK which is used in this project. It is based on Zephyr.

### Installation
To use nRF Connect SDK you need to follow instructions from [nRF Connect SDK docs](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/installation/install_ncs.html#install-prerequisites). 

Basically you need to install nRF Connect SDK extensions into your Visual Studio Code. You should use new Profile in the VS Code otherwise the standard extensions for developing C projects are going to be in conflict.

Then open installed extension and from that zou can download Toolchain. After that you should also download the SDK.

Now it is time to include Cubesat Space Protocol. Download the libcsp library (Cubesat Space Protocol) from the [https://github.com/libcsp/libcsp](https://github.com/libcsp/libcsp).
Now you should navigate to the `ncs` folder where the SDK has been downloaded (under Linux it should be in your home directory). Copy the libcsp library to the `ncs/<sdk-version>/modules/lib` folder. Then you need to change `ncs/<sdk-version>/nrf/west.yml` so open it and find the line with comment "Other third-party repositories". Under it add following two lines:
```
- name: libcsp
  path: modules/lib/libcsp
```

Then open folder `nrf_cubesat` from the VS Code, now you need to create a new build. You need to choose the `nrf52832dk_nrf52832` profile. After that you can build the project and flash the code.

### Code
There are two important functions. First of them is `ble_write_thread` which periodically sends data over BLE. Each 5 s it sends Hello World message with incrementing integer.

Second important function is `bt_receive_cb` which is called when data is received over BLE. Received data are printed into log and if received data is "1" it turns on LED3 or if received data is "0" it turns off LED3.

## Ground Station
Project for the "Ground Station" is located in folder `pc_cubesat`.

### Installation
First of all you have to install Python package called *bleak* which is package for controlling Bluetooth Low Energy. For the installation you can use provided requirements.txt file or run: pip3 install bleak.

Then you have to compile libcsp for Python. You can do that using provided script `install_libcsp.sh`.

### Code
In the project there are three Python files.
1) First file `example_ble_uart.py` is compatible with nRF firmware which has been described above. It prints what nRF sends over NUS (Nordic UART Service) and periodically sends 1 or 0 to toggle the LED3 on the DK board.

2) Second file `example_csp_server_client.py` is example file from the libcsp library. If you followed the installation procedure you should be able to run it. When the address is set to 0, it probably uses loopback interface.

3) Third file `csp_python_binding.py` is a script for testing of a new interface called *basic* which we created. This interface is based on KISS interface which has been already written in original library. KISS interface is a protocol used by hams and is used on top of the UART (thus works with serialized data). \
It would be handy to have an interface with only two functions - write and read. Write function should be called on demand from libcsp and calls Python's BLE send function. Read function should be called from the Python script when data over BLE is received. So we modified KISS interface, we got rid of the UART driver and provided only mentioned functions.\
**The current struggle is with binding to Python. When you run the script, Python will pass pointer to the `tx` function. That pointer to the Python's function lives as an PyObject during initialization of the interface. Libcsp sucessfully calls the Tx function during initialization (row 232 in csp_if_basic.c) but when Python tries to send ping packet we will get a seg fault (row 273 in csp_if_basic.c). It is probably because of the freed memory on the address which is stored in py_tx_func.**

### Modifications of libcsp
As it has been mentioned above, we made few modifications to libcsp. We created new interface called basic which is located in `libcsp/src/initerfaces/csp_if_basic.h`. Corresponding header file is located in `libcsp/include/csp/initerfaces/csp_if_basic.h`. We also had to modify CMake file in `libcsp/src/initerfaces/CMakeLists.txt`.

New interface contains basically three functions:
- csp_basic_add_interface - allocates and sets new interface (already working)
- csp_basic_tx - called by libcsp when it needs to send data over BLE (need to debug segfault)
- csp_basic_rx - called by Python to pass received packet to libcsp (need to create binding to Python)

The last modified file is `libcsp/src/bindings/python/pycsp.c` which provides binding to Python. In this file we *created pycsp_basic_init* function which is binded to Python. Calling this you add new interface and you also pass a pointer to the Tx function which is stored in *PyObject \*py_tx_func*. The *fnc* function is used to calling Python function stored in *py_tx_func*.

## TODO
- [x] Write a report (at least 5 pages)
- [ ] Ground Station side - instead of using loopback in `example_csp_server_client.py` open connection over the NUS. It means get byte array with the packet from libcsp and send it using bleak (function write_gatt_char).
- [ ] Nordic side - Create new interface for using the NUS.

### Possible HINTs
[Examples from libcsp library can help.](https://github.com/libcsp/libcsp/tree/develop/examples)
