# Vertical scale and channel reference repair — 2026-10-28

The rolling timebase previously normalized each history window independently, while the ruler used the frame scale. Pinch gestures also applied a hidden gain on top of manual volts/div. Together these could fill the plot with a sub-volt signal while displaying hundreds of volts.

Frame and rolling plots now share the voltage calibration used by the ruler and voltage cursors. Pinch changes actual volts/div (10 mV/div to 5 V/div), and manual ranges are centered on zero. There is no additional hidden gain.

The “刻度/移 CH1/CH2” selector chooses the channel for the colored voltage ruler, voltage cursors and vertical movement. Each channel has a colored zero-voltage marker. Right-hand measured maximum/minimum, peak-to-peak and RMS remain independent of display offset.

Validation: production UI regression suite passed, including long-history manual scaling, pinch limits, both channel cursor references and offset-independent peak-to-peak values. Native and ARM builds passed. Installed ARM SHA256: 71e7ded5cf42b495138f8e6f38914dc79b6207516c5c374a73aa304449cc4b3a. Actual screen captures at 10 us/div confirm 5 V/div gives a +/-20 V ruler and appropriately small sub-volt traces; AUTO returns to millivolt scaling. Generator settings were not changed. This is a display correction, not a new analog calibration.
