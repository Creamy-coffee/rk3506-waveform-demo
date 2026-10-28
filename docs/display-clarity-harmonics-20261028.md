# Display clarity, RAW-only scope and harmonic receiver — 2026-10-28

The scope always displays the two physical ADC RAW channels. The former scope
demodulation button and automatic takeover by legacy IQ frames are removed.
Modulation configuration belongs to the modulation page. Its selectable list is
OFF, AM, FM, PM, OOK, 2FSK and BPSK. PM has wrapped-phase analysis; it is not a
digital protocol decoder. QPSK has only IQ observation in this codebase, so its
selection is removed until carrier/symbol recovery and bit decisions exist.
Protocol mode numbers retain their original values, including the unused UI ID 4.
Legacy IQ parsing remains available for recordings and NCO diagnostics.

The lock page no longer exposes the obsolete AA5 modulation selector or a
display-only integration-time control. No hardware integration time was being
configured by that dropdown. Qualified MEAS supplies the two-channel phase;
IQ-only phase is relative to the NCO and is never presented as a dual-channel
phase measurement. Invalid qualification, diagnostic flags or stale results
withdraw the phase and the vector together.

## Layout and basic operation

Visible branding is FPGA SCOPE. Wire magic names remain compatible. The scope
header states two 12-bit ADC channels and the 65 MSPS acquisition target, with
actual RAW frame arrival and UI display FPS shown separately. Cursor T1/V1 uses
white, T2/V2 violet; voltages include display gain and DC center. Tick margins
preserve minus signs and both time-axis endpoints. OFF trigger markers are hidden.
The 操作 button explains timebase, shared voltage scale, automatic fitting, reset,
pause, software frame alignment, cursors, pinch zoom and fullscreen. It explicitly
distinguishes software alignment from hardware single-shot capture and explains
the dead time in cross-frame long-timebase history.

Metric values are 18 px bold, system values 20 px bold. Shorter scope metric keys
leave room for two channel values. The image receiver uses a 384×384 canvas with
integer pixel scaling, 20 px status/info, larger mode buttons, and revision-driven
pixel redraw. Received image data remains the only image source for the live page.
Text updates are bounded to 200 ms except for newly received packets/revisions.
Eight generated font subsets cover the new UI strings.

The FFT page reserves separate rows for source/title and touch readout. Its
frequency labels fit inside the graph; dBFS ticks span 0 to -100 dBFS. Touch mapping
uses the same data rectangle as rendering. Frequency axis, cursor samples, peak,
noise estimate and FFT Δf all use the displayed source. FPGA SPEC is CH1 only;
CH2 uses the local RAW FFT. Local Δf uses actual RAW count and effective sampling
rate, not the 8192-point zero-padded interpolation grid. Channel/window selection
also recomputes the cached FFT while paused. Three local windows are retained;
the displayed hardware window is the received three-window protocol enum.
Δf is bin spacing, not an ENBW-adjusted RBW claim or instrument accuracy.

## Sampling and input reliability

An independent FPGA frequency estimate recovers from the old 1 MHz/dec64 alias
trap. Both physical channels constrain the shared RAW sampling rate, so a slow
selected channel cannot hide a faster input. Rate changes clear incompatible
history. Constant/low-AC inputs have a minimum voltage span and do not synthesize
an FFT-bin-one frequency. Short touch reports are delivered one SYN_REPORT at a
time; buffered press and release cannot collapse into one released state. Release
keeps the last position. Covered pages are not continuously repainted while a
dialog is open; the existing UART owner still receives bytes.

## Harmonic integration, default OFF

The receiver is imported from the FPGA-side 16556a03 handoff posted as exchange
message 385. `harmonic_receiver.c/.h` validates 216-byte V32HRM1! v1/v2 packets,
exact 40-bit power consistency and CRC. The existing single UART dispatcher feeds
it; no additional reader/thread owns the device. Linux computes the v2 THD Q24
from exact powers with 64-bit arithmetic; the on-wire v2 zero is not a THD reading.

The FFT sidebar displays the user-specified fundamental reference, H1–H5 in dBc,
THD 2–5, current channel/decimation/effective FFT rate/age and diagnostics. It does
not infer the fundamental from the strongest spectral peak. Configuring requires
the explicit 谐波设置 panel, numeric reference in Hz, channel, HARM ON and Apply.
Commands are one ordered write: F3 OFF, F0 channel, F1 high FCW, F2 low FCW, F3 ON.
They never alter E0–E5 analysis configuration. Repeated results cannot refresh
freshness. Host residence time is added to the FPGA age; after 1500 ms values are
withdrawn. Echo matching uses independent HARM channel/reference and a new result
after the local request baseline; this is not request-ID/ACK causality proof.

Below-noise orders display `< NOISE`; all H2–H5 below noise withdraw formal THD,
rather than displaying zero distortion. Incomplete orders, low resolution,
insufficient fundamental, clipping, overflow, unsupported windows and stale data
remain explicit. Values are marked uncalibrated, including the decimator frontend
response warning when flagged. This is finite-order THD, not THD+N or SINAD.

The running FPGA bit has NOT been changed by this UI release. HARM defaults OFF.
The settings panel names required bit 16556a03 and says to verify its identity
before enabling. Physical harmonic accuracy, frontend calibration and sustained
HRM coexistence acceptance are pending. Synthetic UI screenshots are not physical
harmonic measurements.

## Validation and deployment

Native ASan/UBSan UI regression covers existing modulation/image paths, qualified
and rejected phase, independent RAW/SPEC rates, paused FFT selection, harmonic
power ratios, below-noise results, mismatch/expiry, RAW-only scope and the supported
mode cycle. `tools/run_display_io_test.py` tests every HRM packet split, bad CRC
resynchronization, duplicate freshness, v1/v2, FPGA age and exact F0–F3 command
bytes; it also tests queued Protocol A/B touch press/release reports.

ARMv7 hardfloat static build was tested by ADB on the RK3506 1024×600 panel with
one `/dev/ttyS1` owner at 2 Mbaud. Short 90 ms taps visited scope/FFT/lock/system/
image and opened/closed dialogs. The completed physical capture contains 135
CRC-good RAW frames, zero bad RAW candidates, decimation 1, 1024 samples per
channel. With a 1 MHz sine, least-squares fit R² is about 0.9997; this confirms
waveform shape, not calibrated amplitude accuracy. The source was read back as
CH1=1 MHz/0°, CH2=1 MHz/-90°; the qualified phase screen read 89.99°. Cable/channel
calibration remains outside that short check.

Under the bounded 1 MHz source window, normal UI display was 23.8 FPS and RAW
arrival 5.00 frames/s. The forced full-screen redraw diagnostic was 14.4 FPS and
is a different workload. Neither figure is a full-bandwidth acquisition-rate or
all-pages FPS promise. Generator settings were restored and verified afterwards.

Installed binary SHA256:
`825a3aa69836739f77f918829b02ae1c370993397d0fd7bbb654491209ff25ef`.
Rollback binary: `/usr/bin/waveform_demo.pre_clarity_20261028`.
Host evidence: `firmware/reports/display_clarity_20261028/physical_v3`,
`performance`, `deployed.json`; native validation in
`firmware/experimental/modulation_ui_20261026/clarity_ui_20261028` and
`clarity_io_20261028`. Earlier physical/physical_v2 attempts are diagnostic
failures; they are not used as completed navigation acceptance.

Palette references: [Nord](https://github.com/nordtheme/nord) and
[Catppuccin](https://github.com/catppuccin/catppuccin). Their dark neutral backgrounds
and restrained accents informed the display; no external UI code was copied.
