# Pinned liquid-dsp 1.7.0

Source: https://github.com/jgaeddert/liquid-dsp (release 1.7.0).
Archive SHA256: `33c42ebc2e6088570421e282c6332e899705d42b4f73ebd1212e6a11da714dd4`.
MIT license retained in `liquid-dsp.LICENSE` and the source archive.

`liquid.cmake` extracts offline into the build directory and applies only three
build-system changes there: C-only project; static library; omit the upstream
unconditional SIMD runtime probes. Generic C DSP kernels retain the algorithms
and numeric types; no reduced-precision replacement. Examples/tests/benchmarks
are not bundled into the GUI. No target executable runs during cross configuration.

The dependency is statically linked, so no `libliquid.so` deployment is needed.
The existing ARM SDK toolchain must still build/verify the executable on its target.
Host builds and archive pinning do not constitute RK3506 execution acceptance.

Algorithm/API references:
- https://www.liquidsdr.org/doc/symtrack/
- https://www.liquidsdr.org/doc/resamp/
- `src/framing/src/symtrack.proto.c` in the pinned source.
