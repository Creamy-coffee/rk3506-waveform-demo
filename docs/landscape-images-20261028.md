# Landscape images and stationary scope traces — 2026-10-28

Removed vertical trace dragging and the 1 0V / 2 0V plot labels at the user's request. Voltage scale/cursor channel selection remains, renamed 电压轴 CH1/CH2. Shared manual and rolling voltage calibration remains intact.

The image canvas is now 704 x 384, with status and a wrapped 2 x 2 mode/reset button group on the right. Images use uniform integer nearest-neighbor scaling and centered margins, preserving aspect ratio. Square images remain square. A 128 x 64 image is shown as 640 x 320; the receiver retains the existing 128-pixel per-axis protocol limit and one-bit monochrome format.

The companion Windows sender at firmware/tools/image_sender supports square and 2:1 landscape requests, with an optional height field (legacy requests default to square). Preview/crop geometry, byte/packet estimates, PBM headers and packet dimensions use the selected height. Initial fit retains the whole source image with white margins; intentional zoom can crop. Existing dithering modes are retained. This changes image geometry, not modulation or FPGA RTL.

Validation: ARM/native builds and production LVGL regressions; geometry preview for landscape images; sender unit checks including real rectangular wire headers, byte lengths and rejection of unsupported aspect ratios; all eight existing dither modes; local HTTP/token/JS syntax checks. No generator output was changed for these UI checks. A native layout preview is not a new physical image RF acceptance.

Build environment recovered to FPGA-Build-2404 on D: because the previously registered Ubuntu-24.04 ext4.vhdx was absent. See D:/FPGA_bag/build_environment/RECOVERY_20261028.md.
