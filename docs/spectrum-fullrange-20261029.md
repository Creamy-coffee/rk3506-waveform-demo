# Complete spectrum display span — 2026-10-29

The real-spectrum mapping and plot ticks previously covered only 0..-100 dBFS, although the V32SPEC byte encoding spans 0..-120. Codes below approximately43 were clamped to the bottom. Zero-coded points also lay on the plot border, making the lower trace hard to see. This is a display-range mismatch, separate from the fixed-point resolution limit.

Real FPGA/local spectra now map to a 120 dB display span, with matching 0/-30/-60/-90 ticks and a <RES label at the bottom rather than claiming a measured -120 noise floor. Demo normalization remains100 dB with corresponding ticks. The floor trace is inset two pixels from the frame so its segments remain visible. Spectrum peak/cursor numeric values keep their original dBFS units. True-bin x positions and no-signal validity checks remain.

Added three intermediate frequency labels at quarter Nyquist intervals, generated from the actual active spectrum rate. At 65 MSPS these are 8.125/16.250/24.375 MHz, with the existing 0 and32.500MHz endpoint labels. This clarifies the full positive-frequency extent without inventing additional FFT bins.

Native/ARM builds and production UI regressions are required. Added a CRC-valid SPEC fixture with byte16 (approximately -112.47 dBFS): it must retain positive normalized height16/255 instead of clipping to zero. Existing nonzero peak dBFS, actual-bin position and zero-spectrum cases remain covered. The low-code fixture tests the protocol renderer; it is not an assertion that the current integer FPGA engine resolves that analog input level.

The bit remains unchanged. <RES still denotes below representation/resolution, not calibrated zero noise. Fixed-point low-level spurs and true spectrum measurement limits remain separate from this layout fix.
