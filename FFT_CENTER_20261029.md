# Window-independent center frequency — 2026-10-29

Fixed: RAW spectrum center estimation now always uses a Hann FFT of the same mean-removed frame. Use bounded three-point log-parabolic interpolation. Plot and amplitude continue to use the selected Hann/Blackman/FlatTop window; amplitude is sampled at the estimated component center. Marker position follows this center, not the maximum of a FlatTop plateau or a display bucket. The readout is named 中心频率.

Hann reuses its display FFT. Other windows require one additional FFT and a 16 KiB static magnitude buffer. FPGA bit and UART protocol are unchanged; board remains on a222905e. Hardware SPEC continues its existing bin-based estimate.

Validation:
- C numerical tests passed: existing coherent gain/DC/FlatTop off-bin checks; non-bin single tones including near Nyquist; resolved unequal-amplitude adjacent tones; zero/boundary inputs.
- Native production UI regression passed with ASan/UBSan. Frozen-frame Hann/Blackman/FlatTop center equality and marker/readout alignment are asserted while curves differ. Existing hardware source, channel selection, image/math/trigger/lock/modulation checks remain passing.
- RK3506 deployed binary SHA256 4fcdcfa0cb4facd4485187bac6ef2a72c225020829889f232cf20cfa52576dce.
- Physical generator CH2 to screen CH1: all three windows display 1.000MHz for 1MHz, 1.030MHz for 1.03MHz, and -29.4dBFS amplitude in all six cases. Frozen samples per frequency ensure this is a matched-frame comparison.
- FlatTop live plot screenshots change; returned to live RAW Hann. Generator frequency, mode, waveform, amplitude and output read back equal to test-start values.

Limits: adjacent-tone validation is synthetic, not RF two-tone acceptance. Interpolation improves isolated-tone estimation, not the fs/N ability to resolve closely spaced signals. Center identifies the dominant AC component, not an automatic carrier estimate for arbitrary modulation. Absolute amplitude, ADC timebase and THD remain uncalibrated. No promise of MHz-resolution separation from a 1024-sample frame.
