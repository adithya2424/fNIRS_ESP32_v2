# NIRDuino-v2
Optimized, low-latency version of the NIRDuino fNIRS system with enhanced timing precision, reduced memory usage, and faster dual-intensity acquisition. Built on Arduino Nano ESP32 and Bluetooth-configurable via Android.

# ESP-IDF Installation Guide

## Quick Installation with VS Code

The most straightforward method for installing ESP-IDF is by using the official Visual Studio Code extension.

### Prerequisites

Before proceeding, ensure you have installed all the necessary prerequisites for your operating system as outlined in the [ESP-IDF VS Code extension documentation](https://github.com/espressif/vscode-esp-idf-extension).

### Installation Steps

1.  **Install the VS Code Extension:**
    Search for and install the [ESP-IDF extension](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension) from the Visual Studio Code Marketplace.

2.  **Configure the Extension:**
    * Once installed, open the command palette using `Ctrl+P` or `Cmd+P`.
    * Run the command `ESP-IDF: Configure ESP-IDF extension`.

3.  **Installation Settings:**
    * Select the **Express** installation option.
    * For the `Select ESP-IDF Version:` prompt, choose **v5.1.4**. It is crucial to use this specific version to ensure compatibility with the Arduino Core for ESP32.
    * Click **Install** to begin the download and setup process. The ESP-IDF SDK is approximately 1.4GB.

    > **Note:** If you experience slow download speeds from the default GitHub server, you can abort the installation and restart the configuration process. When prompted, select the **Espressif** server instead for a potentially faster download.

The express installation will download and set up both the ESP-IDF and the necessary toolchains (including GCC and GDB for each ESP32 CPU architecture). On Linux and macOS, the default installation paths are:
* **ESP-IDF:** `$HOME/esp/esp-idf`
* **Toolchains:** `$HOME/.espressif`

While a command-line interface (CLI) installation is available, it is a more complex process and not recommended for beginners.

### macOS Specific Issue: GCC Toolchain Installation

There is a known issue where the VS Code extension may fail to correctly install the GCC toolchains on macOS.

**Verification:**
After the installation process completes, verify if the `xtensa-esp32s3-elf` toolchain directory exists at `~/.espressif/tools/xtensa-esp32s3-elf`.

**Workaround:**
If the directory is missing, open a terminal and execute the following script to manually install the toolchains:

```bash
~/esp/esp-idf/install.sh
```

This issue has been reported to the ESP-IDF VS Code extension developers. You can track its status here: [GitHub Issue #1301](https://github.com/espressif/vscode-esp-idf-extension/issues/1301).

