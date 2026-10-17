# STM32H723 USB Audio Class 1 DAC

A USB Audio Class 1 (UAC1) DAC built on a WeAct **STM32H723VGT6** board. It enumerates
as a driverless USB sound card at **44.1 / 48 / 96 kHz, 24-bit stereo**, and streams the
incoming audio out over **SAI1 block A in I2S Philips format** to an external I2S DAC
module (e.g. PCM5102A). A small on-board ST7735 TFT shows the USB enumeration state.

The USB and SAI plumbing is deliberately kept separate from a single **float32 DSP
hook**, so filters / EQ / effects can be added in one place without touching the USB or
I2S code.

```
Host  ──USB FS──►  UAC1 async OUT EP1 (24-bit stereo)
                        │  decode
                        ▼
                AudioDSP_ProcessBlock(float *pcm, uint32_t nframes)   ← DSP hook
                        │  volume / mute
                        ▼
                32-bit left-justified I2S ring buffer (RAM_D2)
                        │  DMA1_Stream0 (circular)
                        ▼
                SAI1 block A master TX  ──I2S──►  external DAC
```

## Features

* USB Full Speed Audio Class 1 device — no driver installation (Windows / macOS / Linux)
* 24-bit stereo PCM, **Fs = 44.1 / 48 / 96 kHz**
* **Asynchronous** isochronous endpoint with 3-byte feedback (10.14 format) so the
  host adapts its rate to the DAC's `Fs`
* USB Audio **Volume** (0 dB to −96 dB in 3 dB steps) and **Mute** via the Feature Unit
* SAI1 master I2S (Philips standard) output, 32-bit slots / 24-bit significant
* Float32 DSP insertion point (`Core/Src/audio_dsp.c`)
* On-board ST7735 0.96" 160×80 TFT shows USB state
* Bus powered

## Hardware

| Item        | Detail                                                              |
|-------------|---------------------------------------------------------------------|
| Board       | WeAct Studio STM32H723VGT6                                          |
| MCU         | STM32H723VGT6, Cortex-M7, 1 MB flash                                |
| USB         | Full speed USB device mode |
| Audio out   | I2S audio module |
| Display     | ST7735 0.96" 160×80  |

> **Sample rates:** 44.1 / 48 / 96 kHz are advertised. On a rate change the firmware
> reconfigures PLL2 (the SAI1 kernel clock) and re-inits SAI1, and updates the async
> feedback nominal to match the actual generated rate.

## USB DAC function

* **Audio Control interface** — Input Terminal (USB streaming, 2 ch) → Feature Unit
  (Mute + Volume, 0..−96 dB in 3 dB steps) → Output Terminal. Volume and mute are applied
  in the USB callback by scaling the samples.
* **Audio Streaming interface** — alternate setting 1 has two isochronous endpoints:
  * `EP 0x01` OUT, async, 24-bit stereo, `wMaxPacketSize` = 97 frames × 2 ch × 3 B = **582 B** (sized for 96 kHz)
  * `EP 0x81` IN, feedback, 3 bytes (10.14 format), reported every `bRefresh` = 2² = **4 ms**
* **Feedback loop** — every 1 ms SOF (`USBD_AUDIO_SOF`) the firmware reads the SAI DMA
  counter (`DMA1_Stream0` NDTR) to derive the playback read pointer, PID-corrects a nominal
  feedback value (`fb_nom`, selected per rate) and transmits it on EP `0x81`. This keeps the
  host's send rate locked to the actual `Fs`.
* Samples arriving as 24-bit little-endian bytes are decoded to int32, converted to
  float32 in `[-1, 1)`, passed through the DSP hook, scaled by volume/mute, and written as
  left-justified 32-bit words into the ring buffer (`Core/Src/usbd_audio.c`).
* The DMA ring buffer lives in RAM_D2 and is marked non-cacheable via an MPU region, so
  the CPU and the SAI DMA see coherent data (DTCM is not reachable by DMA1).

## I2S output pinout

The SAI1 block A pins are exposed on the WeAct board headers. These are the signals to
connect to an external I2S DAC:

| Board   | PCM5102A | Description                          |
|---------|----------|--------------------------------------|
| 5V      | VCC      | module supply                        |
| GND     | GND      | common ground                        |
| **PE5** | BCK      | bit clock                            |
| **PE6** | DIN      | serial data                          |
| **PE4** | LCK      | word select                          |

## DSP hook

All DSP work goes in one function:

```c
/* Core/Src/audio_dsp.c */
void AudioDSP_ProcessBlock(float *pcm, uint32_t nframes);
```

`pcm` holds interleaved L/R `float32` samples in `[-1.0, 1.0)`. The default implementation
is a pass-through; replace the body with filters, EQ, dynamics, etc. — the USB decode and
SAI playback paths do not need to change. `AudioDSP_Init(sample_rate)` is called when the
stream starts.

## Known limitations

* **Host rate matching.** The async feedback endpoint is USB Audio 1.0 conformant
  (3-byte 10.14, `bRefresh` = 4 ms, see the descriptor audit above), but some hosts do
  **not** perfectly track it. On both macOS and Windows 10 the ring-buffer level drifts
  slowly (tens of ppm) instead of holding steady at the 776-frame setpoint; Windows also
  adds isochronous hiccups that perturb the level. When the level reaches a boundary the
  safe-zone drops a few packets at rollover — inaudible on the bench. This is accepted
  as-is; no firmware-side workaround is applied.

## Debug instrumentation

`-DAUDIO_DEBUG` is enabled in the `Makefile` `C_DEFS` (remove it for a lean production
build). With it on, the on-board LED (**PE3**) lights when the ring buffer enters the safe
zone, and the ST7735 shows the USB state plus `R:<rate> F:<free frames>` and `Drop:<count>`.
## References

* Reference implementation: [har-in-air/STM32F411_USB_AUDIO_DAC](https://github.com/har-in-air/STM32F411_USB_AUDIO_DAC)
* [Dragonman STM32F469 USB Audio](https://github.com/dragonman225/stm32f469-usbaudio)
* WeAct Studio: <https://github.com/WeActStudio/WeActStudio.MiniSTM32H723>
