# FFT view and window selection fix — 2026-10-29

Window clicks previously recalculated the host FFT but the display continued to prefer fixed-Hann V32SPEC frames. The curve therefore did not change. Window clicks now explicitly select RAW FFT. AUTO restores hardware preference and Hann; CH2 uses RAW because V32SPEC only contains ADC CH0 (screen CH1). The title and window readout identify the actual source.

The three exposed windows remain Hann, Blackman and FlatTop. Corrected the FlatTop odd cosine signs and normalized each window by its actual coherent gain, including proper DC scaling. No new FPGA bit is required. Hardware stays on 16556a03.

Display: full positive frequency band, 0 to -120 dBFS at 20 dB intervals, intermediate frequency labels, major grid only, actual trace data. No synthetic noise or extra channel is generated. A -120 dBFS axis limit does not assert instrument dynamic range.

Validation:
- Production native UI regression with AddressSanitizer/UndefinedBehaviorSanitizer passed, including valid hardware SPEC while cycling three RAW windows, paused recomputation, and return to hardware.
- FFT numerical checks passed: coherent amplitude, DC, distant sidelobes, and non-bin-centered FlatTop amplitude (<0.3% numerical error).
- RK3506 deployed binary SHA256: 78486b6dbeeb03051d349d9fb7de2590d84cd003f7d82aeb569f937ef7c44f30.
- ADB touch verified all three windows on a frozen physical RAW frame; captured plot pixels differ. Restored live RAW Hann after checking hardware fallback.
- Generator queried without changing settings: CH2 continuous sine, 1 MHz, 0.3333333 Vpp, output ON. Observed displayed peak approximately 1 MHz.

Limit: at 65 MSPS the current 1024-point RAW frame has fs/N = 63.477 kHz; hardware 4096-point FFT has 15.869 kHz. Zero padding does not improve resolution. Switching windows does not establish precision amplitude/THD calibration. Harmonics remain an independent FPGA result, not recomputed from the selected host window.

Artifacts: firmware/reports/window_fix_ui_20261029 and firmware/experimental/modulation_ui_20261026/window_fix_ui_20261029 (outside this screen repository).
