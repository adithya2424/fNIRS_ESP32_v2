# NIRDuino-v2

Optimized, low-latency version of the NIRDuino fNIRS system with enhanced timing precision, reduced memory usage, and faster dual-intensity acquisition. Built on Arduino Nano ESP32 and Bluetooth-configurable via Android.

## Getting Started: Build & Flash Instructions

This guide provides the steps to clone, build, and flash the NIRDuino-v2 firmware onto your device.

### Prerequisites

  * You must have **ESP-IDF v5.1.4** installed and configured on your system.
  * If you have not yet installed ESP-IDF, please follow the detailed **Appendix: ESP-IDF Installation Guide** at the bottom of this file first.

### 1\. Clone the Repository

Because this project uses other repositories as components (submodules), you must use a special flag when cloning.

Open your terminal and run the following command:

```bash
git clone --recurse-submodules https://github.com/adithya2424/NIRDuino-v2.git
```

> **Why `--recurse-submodules`?** This flag is essential. It tells Git to not only clone this repository but to also automatically download the required components, like `esp-nimble-cpp`. Without it, the project will not compile.

**If you already cloned without the flag:** If you have already cloned the project and forgot the flag, you can fix it by running this command from inside the project directory:

```bash
git submodule update --init --recursive
```

### 2\. Build the Project

Navigate into the newly cloned project directory and use the ESP-IDF command-line tool to build the firmware.

```bash
cd NIRDuino-v2
idf.py build
```

This command will compile the entire project. The first build may take several minutes.

### 3\. Flash and Monitor

1.  **Connect** your Arduino Nano ESP32 to your computer via USB.
2.  **Find your device's port name.**
      * **On Linux/macOS:** It will look like `/dev/ttyUSB0` or `/dev/tty.usbmodem...`
      * **On Windows:** It will look like `COM3`
3.  **Run the `flash` and `monitor` command.** Replace `<PORT>` with your device's actual port name. This will upload the firmware and then immediately show you the serial output from the device.

<!-- end list -->

```bash
idf.py -p <PORT> flash monitor
```

*Example for Linux:* `idf.py -p /dev/ttyUSB0 flash monitor`
*Example for Windows:* `idf.py -p COM3 flash monitor`

To exit the serial monitor, press `Ctrl+]`.

## Appendix: ESP-IDF Installation Guide

This is a one-time setup to prepare your computer for ESP-IDF development.

### Quick Installation with VS Code

The most straightforward method for installing ESP-IDF is by using the official Visual Studio Code extension.

#### Prerequisites

Before proceeding, ensure you have installed all the necessary prerequisites for your operating system as outlined in the [ESP-IDF VS Code extension documentation](https://github.com/espressif/vscode-esp-idf-extension).

#### Installation Steps

1.  **Install the VS Code Extension:**
    Search for and install the [ESP-IDF extension](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension) from the Visual Studio Code Marketplace.

2.  **Configure the Extension:**

      * Once installed, open the command palette using `Ctrl+P` or `Cmd+P`.
      * Run the command `ESP-IDF: Configure ESP-IDF extension`.

3.  **Installation Settings:**

      * Select the **Express** installation option.
      * For the `Select ESP-IDF Version:` prompt, choose **v5.1.4**. It is crucial to use this specific version.
      * Click **Install** to begin the download and setup process.

> **Note:** If you experience slow download speeds, you can abort and restart the installation, selecting the **Espressif** server instead of the default GitHub server.

#### macOS Specific Issue: GCC Toolchain Installation

There is a known issue where the VS Code extension may fail to correctly install the GCC toolchains on macOS.

**Verification:**
After installing, check if the folder `~/.espressif/tools/xtensa-esp32s3-elf` exists.

**Workaround:**
If it does not, open a terminal and execute `~/esp/esp-idf/install.sh`. This should properly unpack the GCC toolchains. This issue has been reported here: [GitHub Issue \#1301](https://github.com/espressif/vscode-esp-idf-extension/issues/1301).