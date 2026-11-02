[Explanation_English.md](https://github.com/user-attachments/files/34254401/Explanation_English.md)
# EXOSYS CartoEngine (Standalone ESP32-S3 Edition - BETA)

Offline vector/raster cartographic rendering engine (`.QOI`) and multi-layer A* heuristic routing solver (`.HWY`) engineered for **ESP32-S3** microcontrollers with PSRAM. It delivers sub-pixel kinetic panning at up to 125 FPS, lock-free IRAM decompression, zero-copy pointer swapping, and multi-kilometer route calculation in seconds within a strict **3.28 MB PSRAM** budget.

## 1. EXOSYS Ecosystem Architecture

`CartoEngine` is the hardware execution stage of a three-tier production pipeline. Rather than parsing raw web maps on the microcontroller, it consumes pre-compiled binary assets tailored for the Xtensa LX7 architecture:

1. **Exosys Mobile App (Android / Kotlin):** Scans target geographic regions via the Overpass API, isolates populated urban footprints (preventing empty rural/mountain tile downloads at high zoom levels `Z15–Z20`), and exports the `.exorecipe.json` job specification. It also acts as a secondary GNSS beacon over Bluetooth Low Energy (BLE) or Wi-Fi.
2. **Exosys PC Forge (Python + Numba C):** Receives the job over LAN (port `8890`) or via a `.JSON` file. It downloads map tiles in parallel, applies hardware-accelerated Numba shaders (`Nocturno V1`, `Arasaka V2`, `Kiroshi V3`, `Pip-Boy 3000 CRT V4`), encodes each tile into the proprietary **QOI-565** binary format packed into $32 \times 32$ cluster files, and compiles OpenStreetMap road networks into `.HWY` binary graphs.
3. **Exosys CartoEngine (ESP32-S3 C++):** Streams clusters from the MicroSD card over high-speed SPI, hydrates the LVGL display matrix in real time from PSRAM, and executes the A* solver across `.HWY` sector files.

## 2. Internal Engineering & PSRAM Memory Management

To eliminate heap fragmentation on embedded targets, `CartoEngine` uses **Dual-Purpose Static PSRAM Arenas**. Upon calling `init_carto_engine()`, contiguous blocks are allocated once in PSRAM and dynamically repurposed depending on the active engine mode:

| PSRAM Bank | Exact Size | Role in Navigation Mode (Map) | Role in Routing Mode (A* Solver) |
| :--- | :--- | :--- | :--- |
| `global_arena` | 2,097,152 B | 16 RGB565 tile buffers (256x256 px) | 44,000 nodes, 120,000 edges, and Hash tables |
| `vault_arena` | 819,200 B | Lock-Free L2 Cache (20 QOI slots) | Local-to-global index map & Binary Min-Heap PQ |
| `shadow_buffer` | 131,072 B | Zero-Copy decompression staging buffer | Static base pointer reserve |
| `static_placeholder` | 131,072 B | 256x256 tactical fallback radar grid | Temporary texture for all 16 screen tiles |
| `compressed_buffer` | 65,536 B | SPI read buffer from MicroSD | Idle during graph computation |
| `hdr_cache` | 32,768 B | 4 cluster header slots (8 KB each) | Invalidated upon route completion |

### 2.1 `.QOI` Cluster Format & `GhostMiner` Task (Core 0)
FAT32/exFAT filesystems suffer severe latency when opening thousands of individual tile files. `CartoEngine` solves this by packing **1,024 tiles** ($32 \times 32$) into a single cluster file `/MAP/Z{z}/C_{cx}_{cy}.QOI`:
* **Direct-Access Header (8,192 bytes):** Contains 1,024 `<uint32_t offset, uint32_t length>` pairs arranged in *Column-Major* order:
  $$\text{Local Index} = (\text{tx} \bmod 32) \times 32 + (\text{ty} \bmod 32)$$
* **Asynchronous Hydration & Zero-Copy Swap:** While Core 1 runs LVGL and kinetic touch physics at 8 ms per tick, the `GhostMiner` task pinned to Core 0 scores pending tiles based on the movement vector `(vector_x, vector_y)`. It reads the compressed QOI payload from the MicroSD, decodes it into `shadow_buffer`, and swaps the `tiles[best_idx].buffer` and `shadow_buffer` pointers inside a critical section (`tile_spinlock`) lasting under 2 microseconds.
* **Negative RAM Caching:** Missing clusters or empty ocean/rural coordinates (`offset < 8192`) are recorded in ring buffers (`sd_blacklist[100]` and `empty_tile_ring[128]`) to prevent redundant SPI bus queries.

### 2.2 `decode_qoi565_fast` Decoder in IRAM
Placed in Instruction RAM (`IRAM_ATTR`), the custom decoder unpacks $256 \times 256$ tiles in ~1.8 ms using three 1-byte opcodes operating directly on native 16-bit RGB565 pixels:
* `0x00 | len`: Run-Length Encoding (RLE) of 1 to 64 identical pixels.
* `0x40 | idx`: Lookup in a 64-entry rolling color hash table indexed via `(px ^ (px >> 3)) & 63`.
* `0x80` + `uint16_t`: Uncompressed 16-bit RGB565 literal pixel.
To avoid unaligned 16-bit write penalties over the Octal/Quad PSRAM bus, pixels are decoded into a fast internal SRAM strip (`sram_strip[1024]`) and flushed to PSRAM in aligned 2,048-byte bursts via `memcpy`.

### 2.3 Multi-Sector A* Routing Engine & Boundary Welder (`.HWY`)
When a route is requested, `CartoEngine` parks `GhostMiner`, points all 16 map tiles to `static_placeholder`, and transforms the 2.91 MB of `global_arena` + `vault_arena` into a high-capacity graph solver:
1. **Elliptical Capsule Filtering:** Discards nodes outside a dynamic corridor radius around the origin-destination segment (`project_point_onto_segment_m2`) on the fly during SD streaming.
2. **Automatic Boundary Welder (42 m):** When a route spans multiple local sectors (`/SECTORS/S_x_y.HWY`, partitioned every $0.02^\circ$), a 32,768-bucket spatial hash table detects border nodes from adjacent files within 42 meters and stitches them with bidirectional edges.
3. **Guaranteed Strong Connectivity:** Synthesizes penalized reverse edges (`1.35x + 4m`) on one-way segments to prevent dead-ends caused by polygon clipping.
4. **3-Pass Escalation Strategy:**
   * *Pass 1:* Tight capsule over local `.HWY` sectors (fast urban routing).
   * *Pass 2:* Wide capsule combining local sectors + `/ARTERIAL.HWY` (geographic detours).
   * *Pass 3:* Long-distance intercity/state routing over `/ARTERIAL.HWY` (up to 55 km anchor radius).

### 2.4 Tri-State GNSS Fusion & Kinetic Physics
The engine parses hardware UART NMEA/UBX streams alongside remote BLE/Wi-Fi telemetry packets, color-coding the active positioning source on the map marker:
* **Blue (`0x007AFF`):** Internal hardware GPS module connected to the ESP32.
* **Yellow (`0xFFD600`):** Remote GPS coordinates injected from the companion mobile app.
* **Neon Magenta (`0xFF00EA`):** Dual-source fusion weighted dynamically by the hardware receiver's satellite HDOP.

## 3. Hardware Requirements

To deploy `CartoEngine` on your own ESP32 hardware, your setup must meet the following specifications:

| Component | Minimum Requirement | Recommended Hardware |
| :--- | :--- | :--- |
| **Microcontroller** | ESP32 / ESP32-S3 @ 240 MHz | **ESP32-S3 Dual-Core @ 240 MHz** |
| **PSRAM Memory** | **4 MB PSRAM (Mandatory)** | **8 MB Octal PSRAM (OPI)** |
| **Storage** | SPI MicroSD card reader | Class 10 / U1 MicroSD (FAT32) @ 25–40 MHz |
| **Display** | Any LVGL 8/9 compatible display | QSPI AMOLED, 8-bit Parallel IPS, or SPI TFT |
| **Input** | 1 or 2 physical GPIO buttons | Capacitive Touch Panel (CST92xx, FT6236, GT911) |
| **GPS Module** | Optional (can use Exploration Mode or BLE) | **U-Blox M10 / M8N / NEO-6M** over UART |

> **TECHNICAL NOTE ON PSRAM:** A standard ESP32 without external PSRAM (~320 KB internal SRAM) **cannot run `CartoEngine`**, as the engine pre-allocates 3.28 MB of graphical and topological arenas at boot. Commercial boards such as the **LilyGo T-Watch Ultra** and **LilyGo T-Display-S3** ship with 8 MB of PSRAM and are supported out of the box.

## 4. Step-by-Step Hardware Adaptation Guide

All hardware-specific pins and screen dimensions are isolated inside **`ExosysBSP.h`**. You do not need to modify `CartoEngineGeneral.cpp`.

### Step 1: Select or Configure Your Board Profile in `ExosysBSP.h`
Open `ExosysBSP.h` and enable one of the 3 built-in profiles at the top of the file:
* **`BOARD_TWATCH_ULTRA`:** Native profile for the LilyGo T-Watch Ultra ($410 \times 502$).
* **`BOARD_TDISPLAY_S3`:** Profile for the LilyGo T-Display-S3 ($170 \times 320$) using an external SPI MicroSD breakout wired to free GPIOs `CS=10, MOSI=11, SCLK=12, MISO=13`.
* **`BOARD_ESP32_GENERIC`:** For custom ESP32-S3 + SPI display builds. Set your screen resolution and GPIO pins:

```cpp
#define BOARD_ESP32_GENERIC
#define EXO_SCREEN_W      320  // Your display width in pixels
#define EXO_SCREEN_H      240  // Your display height in pixels
#define EXO_SD_CS_PIN     5    // MicroSD SPI CS pin
#define EXO_GPS_RX_PIN    16   // ESP32 RX pin wired to GPS TX
#define EXO_GPS_TX_PIN    17   // ESP32 TX pin wired to GPS RX
#define EXO_GPS_BAUD      9600 // GPS baud rate (9600 or 38400)
```

Updating `EXO_SCREEN_W` and `EXO_SCREEN_H` automatically recalculates `EXO_MAP_ORIGIN_X`, `EXO_MAP_ORIGIN_Y`, and `EXO_MODAL_W` so the $1024 \times 1024$ tile matrix and all popup modals center and scale to your display.

### Step 2: Hook Up Your Touch Driver (Optional)
If using a custom touch controller, locate `exo_touch_get_points` at the bottom of `ExosysBSP.h` and forward your touch library's read call:

```cpp
static inline uint8_t exo_touch_get_points(int16_t *x_arr, int16_t *y_arr, uint8_t max_pts) {
    uint16_t tx, ty;
    if (tft.getTouch(&tx, &ty)) {
        x_arr[0] = (int16_t)tx;
        y_arr[0] = (int16_t)ty;
        return 1;
    }
    return 0;
}
```

### Step 3: Configure `platformio.ini`
Enable PSRAM and include the required libraries (`lvgl`, `SdFat`, `TinyGPSPlus`) in your `platformio.ini`:

```ini
[env:esp32s3_carto]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
board_build.arduino.memory_type = qio_opi ; Use qio_qspi for Quad PSRAM modules
build_flags = 
    -DBOARD_HAS_PSRAM
    -DBOARD_ESP32_GENERIC
    -O3
lib_deps =
    lvgl/lvgl @ ^8.4.0
    greiman/SdFat @ ^2.2.2
    mikalhart/TinyGPSPlus @ ^1.0.3
```

### Step 4: Prepare the MicroSD Card
1. Format a MicroSD card as **FAT32**.
2. Generate your map and road network using **Exosys Mobile App** + **Exosys PC Forge**.
3. Use Step 4 in **Exosys PC Forge** to deploy the assets to your MicroSD card. The root directory will contain:
   * `/MAP/Z6 ... Z20/C_x_y.QOI` (Map tile clusters).
   * `/ARTERIAL.HWY` (Backbone highway graph).
   * `/SECTORS/S_x_y.HWY` ($0.02^\circ$ local street sector graphs).
   * `/UI/placeholder.qoi` *(Optional: custom $256 \times 256$ fallback tile).*
4. Insert the MicroSD card into your ESP32-S3 and power on the board.

## 5. Runtime Controls
* **1-Finger Drag:** Sub-pixel kinetic panning with exponential decay.
* **2-Finger Pinch-to-Zoom:** Jumps across available zoom levels on the SD card after verifying local tile coverage.
* **Double-Tap:** Toggles between **Navigation Mode** (auto-follow GNSS) and **Exploration Mode** (central crosshair, Waypoint manager, and A* route planner).
* **2-Second Long Press:** Re-anchors the viewport to the active GNSS coordinate.
* **5-Second Long Press:** Opens the **Spatial HUD (OSD)** displaying satellite count, HDOP, active source, and U-Blox constellation switcher (`GLONASS` vs `BEIDOU`).
* **Physical Back Button (`EXO_BTN_BACK_PIN`):** Closes active modals, cancels waypoint injection, or returns from Exploration Mode to GNSS tracking.

## License
Distributed under the **GNU General Public License v3.0 (GPLv3)**. See `LICENSE` for details.
