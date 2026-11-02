# EXOSYS CartoEngine (Standalone ESP32-S3 Edition)

Motor de renderizado cartográfico vectorial/ráster offline (`.QOI`) y enrutamiento heurístico A* multicapa (`.HWY`) diseñado para microcontroladores **ESP32-S3** con PSRAM. Ejecuta desplazamiento cinético sub-píxel hasta a 125 FPS, descompresión en IRAM sin bloqueo (Lock-Free), intercambio de punteros Zero-Copy y cálculo de rutas de decenas de kilómetros en segundos utilizando exactamente **3.28 MB de PSRAM**.

## 1. Arquitectura del Ecosistema EXOSYS

`CartoEngine` es el eslabón de ejecución en hardware dentro de una cadena de producción de tres etapas. No descarga mapas de internet directamente; consume binarios precompilados y optimizados para la arquitectura Xtensa LX7:

1. **Exosys Mobile App (Android / Kotlin):** Escanea la zona geográfica mediante Overpass API, aísla manchas urbanas (evitando descargar áreas despobladas en zooms altos `Z15–Z20`) y genera el archivo de orden `.exorecipe.json`. Además, funciona como baliza GNSS secundaria por Bluetooth Low Energy (BLE) o Wi-Fi.
2. **Exosys PC Forge (Python + Numba C):** Recibe la orden del teléfono por red local (puerto `8890`) o archivo `.JSON`. Descarga las teselas en paralelo, aplica shaders de hardware en Numba (`Nocturno V1`, `Arasaka V2`, `Kiroshi V3`, `Pip-Boy 3000 CRT V4`), comprime cada imagen al formato binario propietario **QOI-565** agrupado en clústeres de $32 \times 32$, y compila la topología vial de OpenStreetMap en grafos binarios `.HWY`.
3. **Exosys CartoEngine (ESP32-S3 C++):** Lee la tarjeta MicroSD por SPI de alta velocidad, hidrata la interfaz gráfica LVGL en tiempo real desde la PSRAM y ejecuta el solucionador matemático A* sobre los archivos `.HWY`.

## 2. Ingeniería Interna y Gestión de Memoria (PSRAM)

El motor evita la fragmentación del heap en sistemas embebidos mediante **Asignación de Arenas Estáticas de Uso Dual**. Al arrancar `init_carto_engine()`, se reservan bloques contiguos en PSRAM que cambian de función según el estado del sistema:

| Banco PSRAM | Tamaño Exacto | Uso en Modo Navegación (Mapa) | Uso en Modo Cálculo de Ruta (A*) |
| :--- | :--- | :--- | :--- |
| `global_arena` | 2,097,152 B | 16 buffers RGB565 de 256x256 px | 44,000 nodos, 120,000 aristas y tablas Hash |
| `vault_arena` | 819,200 B | Caché L2 Lock-Free (20 ranuras QOI) | Mapeo local a global y Cola de Prioridad (Min-Heap) |
| `shadow_buffer` | 131,072 B | Buffer de descompresión Zero-Copy | Reserva estática de puntero base |
| `static_placeholder` | 131,072 B | Retícula táctica de respaldo 256x256 | Imagen temporal de las 16 teselas en pantalla |
| `compressed_buffer` | 65,536 B | Buffer de lectura SPI desde MicroSD | Inactivo durante cálculo matemático |
| `hdr_cache` | 32,768 B | 4 cabeceras de clústeres (8 KB c/u) | Invalidado al terminar la ruta |

### 2.1 Formato de Clústeres `.QOI` y Tarea `GhostMiner` (Core 0)
Los sistemas de archivos FAT32/exFAT sufren latencias graves al abrir miles de archivos pequeños. `CartoEngine` resuelve esto agrupando **1,024 teselas** ($32 \times 32$) en un solo archivo de clúster `/MAP/Z{z}/C_{cx}_{cy}.QOI`:
* **Cabecera de Acceso Directo (8,192 bytes):** Contiene 1,024 pares `<uint32_t offset, uint32_t length>` ordenados en formato *Column-Major*:
  $$\text{Índice Local} = (\text{tx} \bmod 32) \times 32 + (\text{ty} \bmod 32)$$
* **Hidratación Asíncrona y Zero-Copy Swap:** Mientras el Núcleo 1 (Core 1) ejecuta LVGL y la física táctil a 8 ms por tick, la tarea `GhostMiner` en el Núcleo 0 calcula qué teselas están entrando al viewport según el vector de velocidad `(vector_x, vector_y)`. Lee el bloque QOI desde la MicroSD, lo descomprime en `shadow_buffer` y, dentro de una sección crítica de menos de 2 microsegundos (`tile_spinlock`), intercambia los punteros entre `tiles[best_idx].buffer` y `shadow_buffer`.
* **Caché Negativa en RAM:** Si un clúster no existe en la SD o una coordenada corresponde a mar/campo vacío (`offset < 8192`), se registra en los anillos `sd_blacklist[100]` y `empty_tile_ring[128]` para no volver a interrogar el bus SPI.

### 2.2 Decodificador `decode_qoi565_fast` en IRAM
Alojado en memoria de instrucciones (`IRAM_ATTR`), decodifica teselas de $256 \times 256$ píxeles en ~1.8 ms empleando solo 3 opcodes de 1 byte sobre palabras RGB565 nativas:
* `0x00 | len`: Racha RLE de 1 a 64 píxeles idénticos al anterior.
* `0x40 | idx`: Acierto en tabla Hash de 64 colores recientes con función de dispersión `(px ^ (px >> 3)) & 63`.
* `0x80` + `uint16_t`: Píxel literal de 16 bits.
Para evitar la penalización de escritura de 16 bits en el bus octal/quad de la PSRAM, el decodificador ensambla tiras de 1,024 píxeles en la SRAM interna rápida (`sram_strip[1024]`) y las descarga hacia la PSRAM en ráfagas alineadas de 2,048 bytes mediante `memcpy`.

### 2.3 Motor de Ruteo A* Multi-Sector y Soldado Fronterizo (`.HWY`)
Cuando el usuario solicita una ruta, `CartoEngine` estaciona a `GhostMiner`, apunta las 16 teselas de pantalla al `static_placeholder` y convierte los 2.91 MB de `global_arena` + `vault_arena` en un solucionador de grafos masivo:
1. **Filtrado por Cápsula Elíptica:** Descarta en tiempo de lectura los nodos de la MicroSD que caen fuera del radio de desviación respecto al segmento origen-destino (`project_point_onto_segment_m2`).
2. **Soldador Automático de Fronteras (42 m):** Cuando una ruta cruza varios sectores locales (`/SECTORS/S_x_y.HWY`, divididos cada $0.02^\circ$), una tabla Hash espacial de 32,768 cubetas detecta nodos fronterizos de archivos distintos separados por menos de 42 metros y los cose dinámicamente con aristas bidireccionales.
3. **Conectividad Fuerte Garantizada:** Genera aristas inversas sintéticas penalizadas (`1.35x + 4m`) en calles sin retorno aparente para evitar callejones sin salida por cortes de polígono.
4. **Escalonamiento en 3 Pasadas:**
   * *Pasada 1:* Cápsula ajustada sobre sectores locales `.HWY` (ruteo urbano rápido).
   * *Pasada 2:* Cápsula amplia combinando sectores locales + `/ARTERIAL.HWY` (rodeos geográficos).
   * *Pasada 3:* Ruteo interurbano/estatal de larga distancia sobre `/ARTERIAL.HWY` (hasta 55 km de radio de anclaje).

### 2.4 Fusión GNSS Tri-Estado y Física Cinética
El motor procesa tramas NMEA/UBX por hardware UART y paquetes remotos por BLE/Wi-Fi, mostrando el estado en el marcador del mapa:
* **Azul (`0x007AFF`):** Posicionamiento exclusivo del módulo GPS físico conectado al ESP32.
* **Amarillo (`0xFFD600`):** Posicionamiento inyectado desde el teléfono móvil.
* **Magenta Neón (`0xFF00EA`):** Fusión dual activa ponderada según el HDOP satelital del reloj frente al teléfono.

## 3. Requisitos de Hardware para Usuarios

Para implementar `CartoEngine` en tu propio proyecto con ESP32, tu hardware debe cumplir las siguientes especificaciones:

| Componente | Requisito Mínimo | Hardware Recomendado |
| :--- | :--- | :--- |
| **Microcontrolador** | ESP32 / ESP32-S3 a 240 MHz | **ESP32-S3 Dual-Core 240 MHz** |
| **Memoria PSRAM** | **4 MB PSRAM (Obligatorio)** | **8 MB Octal PSRAM (OPI)** |
| **Almacenamiento** | Lector MicroSD por bus SPI | MicroSD Clase 10 / U1 (FAT32) a 25–40 MHz |
| **Pantalla** | Cualquier pantalla compatible con LVGL 8/9 | AMOLED QSPI, IPS Paralela 8-bit o TFT SPI |
| **Entrada** | 1 o 2 botones físicos GPIO | Panel táctil capacitivo (CST92xx, FT6236, GT911) |
| **Módulo GPS** | No requerido (usa Modo Exploración o BLE) | **U-Blox M10 / M8N / NEO-6M** por UART |

> **ADVERTENCIA TÉCNICA SOBRE PSRAM:** Un ESP32 estándar sin PSRAM (que solo posee ~320 KB de SRAM interna) **no puede ejecutar `CartoEngine`**, ya que el motor aloca 3.28 MB de memoria gráfica y topológica en el arranque. Placas comerciales listas para usar como **LilyGo T-Watch Ultra** y **LilyGo T-Display-S3** incluyen 8 MB de PSRAM de fábrica y son 100% compatibles.

## 4. Guía Paso a Paso de Adaptación a Otro Hardware

Toda la configuración específica de hardware está aislada en un único archivo: **`ExosysBSP.h`**. No necesitas modificar ni una línea de `CartoEngine.cpp`.

### Paso 1: Configurar tu placa en `ExosysBSP.h`
Abre `ExosysBSP.h` y selecciona o edita uno de los 3 perfiles disponibles al inicio del archivo:
* **`BOARD_TWATCH_ULTRA`:** Configuración nativa para LilyGo T-Watch Ultra ($410 \times 502$).
* **`BOARD_TDISPLAY_S3`:** Configuración para LilyGo T-Display-S3 ($170 \times 320$) con lector MicroSD externo cableado a los pines libres `CS=10, MOSI=11, SCLK=12, MISO=13`.
* **`BOARD_ESP32_GENERIC`:** Para cualquier otro ESP32-S3 con pantalla SPI. Define la resolución exacta de tu pantalla y tus pines:

```cpp
#define BOARD_ESP32_GENERIC
#define EXO_SCREEN_W      320  // Ancho de tu pantalla en píxeles
#define EXO_SCREEN_H      240  // Alto de tu pantalla en píxeles
#define EXO_SD_CS_PIN     5    // Pin CS de tu lector MicroSD
#define EXO_GPS_RX_PIN    16   // Pin RX conectado al TX de tu GPS
#define EXO_GPS_TX_PIN    17   // Pin TX conectado al RX de tu GPS
#define EXO_GPS_BAUD      9600 // Baudios de tu módulo GPS (9600 o 38400)
```

Al cambiar `EXO_SCREEN_W` y `EXO_SCREEN_H`, las macros `EXO_MAP_ORIGIN_X`, `EXO_MAP_ORIGIN_Y` y `EXO_MODAL_W` recalcularán automáticamente el centro geométrico de la malla de $1024 \times 1024$ y reescalarán todas las ventanas emergentes para que no se salgan de tu pantalla.

### Paso 2: Conectar el Touch (Opcional)
Si tu pantalla tiene panel táctil, ubica la función `exo_touch_get_points` al final de `ExosysBSP.h` y haz que llame a la función de lectura de tu librería (por ejemplo, `tft.getTouch(&x, &y)`):

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

### Paso 3: Configurar `platformio.ini`
Asegúrate de habilitar la PSRAM y las librerías requeridas (`lvgl`, `SdFat`, `TinyGPSPlus`) en tu archivo `platformio.ini`:

```ini
[env:esp32s3_carto]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
board_build.arduino.memory_type = qio_opi ; Usar qio_qspi si tu PSRAM es Quad en vez de Octal
build_flags = 
    -DBOARD_HAS_PSRAM
    -DBOARD_ESP32_GENERIC
    -O3
lib_deps =
    lvgl/lvgl @ ^8.4.0
    greiman/SdFat @ ^2.2.2
    mikalhart/TinyGPSPlus @ ^1.0.3
```

### Paso 4: Preparar la Tarjeta MicroSD
1. Formatea una tarjeta MicroSD en **FAT32**.
2. Genera tu mapa y red vial con **Exosys Mobile App** + **Exosys PC Forge**.
3. En el Paso 4 de **Exosys PC Forge**, transfiere el paquete a la MicroSD. La raíz de la tarjeta quedará con esta estructura exacta:
   * `/MAP/Z6 ... Z20/C_x_y.QOI` (Clústeres de imágenes del mapa).
   * `/ARTERIAL.HWY` (Red carretera troncal).
   * `/SECTORS/S_x_y.HWY` (Malla de calles locales subdividida cada $0.02^\circ$).
   * `/UI/placeholder.qoi` *(Opcional: imagen personalizada de $256 \times 256$ para zonas sin cobertura).*
4. Inserta la MicroSD en el ESP32-S3 y enciende el dispositivo.

## 5. Controles en Tiempo de Ejecución
* **Arrastre con 1 dedo:** Desplazamiento con inercia exponencial sub-píxel.
* **Pellizco con 2 dedos (Pinch-to-Zoom):** Salto entre niveles de zoom disponibles en la MicroSD con auditoría previa de cobertura.
* **Doble Toque (Double-Tap):** Alterna entre **Modo Navegación** (centrado automático en GNSS) y **Modo Exploración** (mira central activa, menú de Waypoints y calculador de rutas A*).
* **Pulsación Larga de 2 segundos:** Re-ancla el mapa a la coordenada GNSS actual.
* **Pulsación Larga de 5 segundos:** Abre el **HUD Espacial (OSD)** con telemetría de satélites, HDOP, fuente activa y conmutador de constelaciones U-Blox (`GLONASS` vs `BEIDOU`).
* **Botón Físico (`EXO_BTN_BACK_PIN`):** Cierra menús activos, cancela selección de waypoints o regresa del Modo Exploración al seguimiento GNSS.

## Licencia
Distribuido bajo la licencia **GNU General Public License v3.0 (GPLv3)**. Consulta el archivo `LICENSE` para más detalles.