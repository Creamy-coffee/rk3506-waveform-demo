# Spectrum functional verification and geometry — 2026-10-29

Physical source tests: 0.3333333 Vpp sine at 250 kHz, 1 MHz, 5 MHz and 10 MHz; 1 MHz square; output off. Generator CH2 feeds screen CH1. Each setting settled for 2.5 seconds; actual DRM framebuffer and live widget labels saved. Frequency readouts: 253.906 kHz, 999.756 kHz, 4.999 MHz and 9.998 MHz, consistent with the 4096-point 65 MHz nearest-bin grid. Square spectra change to multiple harmonics and output-off clears signal. The readouts refresh with the input.

Found and repaired:
- Max pooling of 2048 bins into 256 equally spaced display points moved narrow peaks toward their group start. FPGA spectrum display now uses up to 1024 points; each maximum carries its actual normalized bin coordinate into plot.c. Peak arrow uses the actual peak bin. Same-pixel FFT segments are rendered vertically so denser samples cannot silently discard a peak. Scope trace rendering is unchanged. Local/demo FFTs explicitly clear the optional coordinate mapping.
- A completely zero-coded spectrum still drew a peak at the bottom and reported -100 dBFS from a clipped plotting coordinate. Explicit non-DC peak validity now hides the arrow and reports frequency unavailable / level < RES. The prior below-resolution floor/cursor semantics remain.

Native production UI tests cover peak coordinates and a CRC-valid all-zero SPEC packet, alongside existing amplitude/cursor/channel/image/math/trigger regressions. Native and ARM builds pass. Candidate installed and the same six physical conditions repeated. Image analysis of the actual post-fix plot data area (x=71, xspan=695) finds primary-peak glyph locations within 1.5 pixels of the frequency-axis position across all four sine inputs; this includes line/triangle pixel thickness, not a calibrated frequency error bound. Source settings restored with exact readback equality.

Raw before/after screenshots, labels and source receipts: firmware/reports/spectrum_functional_20261029 and spectrum_functional_post_20261029. This is screen display verification; it does not remove the existing minimum-integer-power FFT spurs or establish analog noise/distortion accuracy. No FPGA bit change.
