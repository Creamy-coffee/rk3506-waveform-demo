# Spectrum zero-code display — 2026-10-29

Actual screen inspection: 1 MHz continuous sine, 0.3333333 Vpp, screen CH1 full-span FPGA spectrum. Peak 999.756 kHz is consistent with the nearest 4096-point bin; the 0..32.5 MHz frequency range correctly places 1 MHz near the left edge.

After convergent FFT rounding, many fixed-point power bins are exactly zero. V32SPEC encodes zero as byte 0, whose numeric transport floor is -120 dBFS. That transport floor is not a measured analog noise floor. The median-of-byte histogram incorrectly presented it as -120.0 dBFS. Zero-median FPGA spectra now report < RES; FPGA cursor bins with code 0 also report < RES. Nonzero median and cursor codes keep their numeric dBFS readings. The plot note explains zero-code points are below resolution. No spectrum points are removed or fabricated and the frequency axis/FPGA bit remain unchanged.

The weak peak near 31.5 MHz can also be reproduced in a noise-free fixed-point model: ADC sine amplitude70, phase0.23, 1MHz gives a nonfundamental peak at bin1986 / 31.516MHz with integer power1; phases0 and1 produce no nonfundamental nonzero bins in that bounded model. This demonstrates a numerical mechanism but does not prove the physical peak has no analog contribution. It must not be claimed as calibrated input spurious performance. Improving the underlying low-level FFT precision would require separate RTL and resource/timing verification.

Validation: native production-UI tests exercise normal nonzero SPEC medians/cursor readouts, then actual CRC-valid zero-median SPEC packets and zero-code cursor bins. ARM build and existing regressions pass. Updated fonts include the resolution note. The screenshot and generator readback used to diagnose this are under firmware/reports/spectrum_display_20261029.
