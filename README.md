# STM32H723 USB Audio Class 1 DAC

A USB Audio Class 1 (UAC1) DAC built on a WeAct **STM32H723VGT6** board.

## Features

* USB Full Speed Audio Class 1 device — no driver installation (Windows / macOS / Linux)
* 24-bit stereo PCM, **Fs = 44.1 / 48 / 96 kHz**
* Software **volume control** (0 dB to −96 dB in 3 dB steps) and **Mute**
* I2S (Philips standard) output using STM32H7's SAI1 in master mode
* DSP buffer in float32 (`Core/Src/audio_dsp.c`) for audio processing
* **3-band EQ** (low-shelf / peaking / high-shelf, ±15 dB)
* UI consists of an on-board ST7735 **0.96" 160×80 TFT** which shows the sample rate and
  either an 8-bar FFT spectrum or a 3-band EQ graph, and a **rotary encoder** to control
  the EQ
* Bus powered

## Hardware

| Item      | Detail                                     |
| --------- | ------------------------------------------ |
| Board     | WeAct Studio STM32H723VGT6                 |
| MCU       | STM32H723VGT6, Cortex-M7, 1 MB flash       |
| USB       | Full speed USB device mode                 |
| Audio out | I2S audio module                           |
| Display   | ST7735 0.96" 160×80                        |
| Controls  | Rotary encoder with integrated push button |

> **Sample rates:** 44.1 / 48 / 96 kHz are advertised. On a rate change the firmware
> reconfigures PLL2 (the SAI1 kernel clock) and re-inits SAI1, and updates the async
> feedback nominal to match the actual generated rate.

## GPIO pinout

The I2S audio module and rotary encoder connect to the WeAct board headers as follows:

| Board   | I2S audio module | Rotary encoder | Description                    |
| ------- | ---------------- | -------------- | ------------------------------ |
| 5V      | VCC              |                | power supply to audio module   |
| GND     | GND              |                | common ground                  |
| **PE5** | BCK              |                | bit clock                      |
| **PE6** | DIN              |                | serial data                    |
| **PE4** | LCK              |                | word select                    |
| 3.3V    |                  | V+             | power supply to rotary encoder |
| GND     |                  | GND            | common ground                  |
| **PA0** |                  | S1             | output 1                       |
| **PA1** |                  | S2             | output 2                       |
| **PA2** |                  | KEY            | button                         |

## USB DAC function

All audio processing happens in a single **float32 DSP hook** (a 3-band EQ), so the USB
and SAI plumbing stays separate from the DSP.

```
Host  ──USB FS──►  UAC1 async OUT EP1 (24-bit stereo)
                        │  decode
                        ▼
                AudioDSP_ProcessBlock(float *pcm, uint32_t nframes)   ← 3-band EQ
                        │  (also taps a mono downmix for the spectrum)
                        │  volume / mute
                        ▼
                32-bit left-justified I2S ring buffer (RAM_D2)
                        │  DMA1_Stream0 (circular)
                        ▼
                SAI1 block A master TX  ──I2S──►  external DAC
```

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

## DSP hook

All audio processing goes through one function:

```c
/* Core/Src/audio_dsp.c */
void AudioDSP_ProcessBlock(float *pcm, uint32_t nframes);
```

`pcm` holds interleaved L/R `float32` samples in `[-1.0, 1.0)`. The implementation runs a
**3-band EQ** (three RBJ biquads in transposed direct-form II, one set of state per
channel) and then taps a mono downmix for the spectrum. The USB decode and SAI playback
paths do not need to change to alter the processing.

The biquad coefficients are derived from Robert Bristow-Johnson's *Audio EQ Cookbook*
(low-shelf, peaking and high-shelf forms) — see [References](#references).

| Band     | Filter     | Centre / corner | Q             | Frequency range affected                                           |
| -------- | ---------- | --------------- | ------------- | ------------------------------------------------------------------ |
| **Low**  | low-shelf  | 200 Hz          | — (slope S=1) | DC – ~200 Hz (full gain at DC, settling to 0 dB above the corner)  |
| **Mid**  | peaking    | 1 kHz           | 0.707         | ≈ 520 Hz – 1.9 kHz (half-gain points; ≈ 1.4 kHz wide)              |
| **High** | high-shelf | 4 kHz           | — (slope S=1) | ~4 kHz – Nyquist (0 dB below the corner, full gain toward Nyquist) |

Each band is adjustable over **−15 … +15 dB** in 1 dB steps. The shelf bands use the
cookbook's slope `S = 1`; each shelf reaches its full gain at DC/Nyquist and is centred
(half-gain) on its corner frequency. The high-shelf's upper limit scales with `Fs`
(Nyquist = 22.05 / 24 / 48 kHz at 44.1 / 48 / 96 kHz); the other bands are fixed.

`AudioDSP_Init(sample_rate)` is called when the stream starts and on every rate change: it
rebuilds the Hann window, rebuilds the EQ coefficients for the new rate (the band gains
persist), and resets the filter and analysis state.

The EQ is controlled from the main loop:

```c
void   AudioDSP_SetEqGainDb(uint8_t band, int8_t db);  /* band 0=Low, 1=Mid, 2=High */
int8_t AudioDSP_GetEqGainDb(uint8_t band);             /* −15 … +15 dB */
```

`AudioDSP_SetEqGainDb()` reads the active sample rate and commits the new coefficients
inside a single critical section, so it is atomic with respect to the USB audio ISR (and
against a concurrent rate-change `AudioDSP_Init()`).

## Spectrum display

The 160×80 ST7735 shows:

* **top 20 px** — `STM32H7 DAC` and the active rate (`44.1K` / `48K` / `96K`) as white
  text on a rate-coloured background (blue = 44.1k, green = 48k, red = 96k)
* **lower 60 px** — either an 8-bar FFT spectrum (EQ-style) of the incoming audio with a
  1-px baseline at the bottom of each bar, **or** a 3-band EQ line graph (see
  [3-band EQ](#3-band-eq))

The spectrum is a **display-only analysis tap**; it does not alter the audio. Each block's
mono downmix (post-EQ, pre-volume) is accumulated in `AudioDSP_ProcessBlock()` into a
512-sample ping-pong buffer; `AudioDSP_Spectrum()`, polled from the main loop, then applies
a Hann window, a radix-2 FFT, and bins the result into 8 octave bands mapped to 0..1 over a
−60..0 dB range.

The bands are log-spaced by **FFT bin index** (each is one octave up to Nyquist), so a given
bar sits at a fixed fraction of `fs/2` and its absolute frequency scales with the sample
rate. Bin `k` corresponds to `k · fs / 512`; band `b ≥ 1` covers bins `[2^b, 2^(b+1))`
(band 0 is bin 1 only):

| Bar | Bin range | Freq (× fs/512) | @44.1 kHz   | @48 kHz     | @96 kHz     |
| --- | --------- | --------------- | ----------- | ----------- | ----------- |
| 0   | 1         | fs/512          | 86 Hz       | 94 Hz       | 188 Hz      |
| 1   | 2–3       | fs/256 – fs/128 | 172–344     | 188–375     | 375–750     |
| 2   | 4–7       | fs/128 – fs/64  | 344–689     | 375–750     | 750–1500    |
| 3   | 8–15      | fs/64 – fs/32   | 689–1378    | 750–1500    | 1500–3000   |
| 4   | 16–31     | fs/32 – fs/16   | 1378–2756   | 1500–3000   | 3000–6000   |
| 5   | 32–63     | fs/16 – fs/8    | 2756–5513   | 3000–6000   | 6000–12000  |
| 6   | 64–127    | fs/8 – fs/4     | 5513–11025  | 6000–12000  | 12000–24000 |
| 7   | 128–255   | fs/4 – fs/2     | 11025–22050 | 12000–24000 | 24000–48000 |

## 3-band EQ

A mechanical rotary encoder (TIM2 quadrature mode, PA0 = TI1 / PA1 = TI2) with a
push button (PA2, active-low) controls a 3-band EQ:

* **Turn** — in EQ mode, adjusts the selected band by ±1 dB (range −15…+15 dB, 31 steps).
* **Press** — cycles through: spectrum → **Low** → **Mid** → **High** → spectrum.

In EQ mode, the selected band is highlighted by a solid marker. Bands are RBJ biquads
applied to the audio: low-shelf 200 Hz, peaking 1 kHz (Q 0.707), high-shelf 4 kHz, ±15 dB,
recomputed for the active sample rate. Gains persist across sample-rate changes but not
across power cycles.

The encoder input is implemented in `Bsp/bsp_encoder.c`. Polarity and sensitivity are
compile-time tunables in `Bsp/bsp_encoder.h` (`BSP_ENC_DIR_SIGN`,
`BSP_ENC_COUNTS_PER_DETENT`, `BSP_ENC_BUTTON_ACTIVE_LOW`).

## Known limitations

* **Host rate matching.** The async feedback endpoint is USB Audio 1.0 conformant
  (3-byte 10.14, `bRefresh` = 4 ms, see *USB DAC function*), but some hosts do
  **not** perfectly track it. On both macOS and Windows 10 the ring-buffer level drifts
  slowly (tens of ppm) instead of holding steady at the 776-frame setpoint; Windows also
  adds isochronous hiccups that perturb the level. When the level reaches a boundary the
  safe-zone drops a few packets at rollover — inaudible on the bench. This is accepted
  as-is; no firmware-side workaround is applied.

## Debug instrumentation

The `AUDIO_DEBUG` code is retained but **disabled by default** — add `-DAUDIO_DEBUG` to
`C_DEFS` in the `Makefile` for a debug build (regeneration reverts this). With it enabled,
the on-board LED (**PE3**) lights when the ring buffer enters the safe zone, and the ST7735
additionally prints `R:<rate> F:<free frames>` / `Drop:<count>` (overlapping the spectrum
area when the spectrum is active). With it disabled (default), the LED stays off and the
FFT spectrum is shown.

## References

* RBJ *Audio EQ Cookbook* (biquad EQ coefficient formulas used by `Core/Src/audio_dsp.c`):
  <https://www.w3.org/TR/audio-eq-cookbook/>
* Reference implementation: [har-in-air/STM32F411_USB_AUDIO_DAC](https://github.com/har-in-air/STM32F411_USB_AUDIO_DAC)
* [Dragonman STM32F469 USB Audio](https://github.com/dragonman225/stm32f469-usbaudio)
* WeAct Studio: <https://github.com/WeActStudio/WeActStudio.MiniSTM32H723>
