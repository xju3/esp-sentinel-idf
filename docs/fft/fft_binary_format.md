# FFT binary upload contract

## Task and upload

- The server sends `{ "action": 99, "val": 0 }`.
- `action=99` only requests one FFT acquisition. It does not encode FFT size,
  sampling interval, or sensor range.
- The device chooses the acquisition size from its speed/revolution policy and
  chooses the range locally.
- The device uploads the binary object with the task UUID as the object name.

## Current format (v1)

All numbers are little-endian.

| Offset | Size | Type | Meaning |
| --- | ---: | --- | --- |
| 0 | 16 | bytes | Device/SN hint; not used as database identity because it may be truncated |
| 16 | 4 | `uint32` | Unix timestamp in seconds |
| 20 | 4 | `uint32` | Time-domain FFT size `N` |
| 24 | 4 | `float32` | Sampling rate in Hz |
| 28 | 4 | `uint32` | Range in g |
| 32 | `N/2 * 4` | `float32[]` | X positive-frequency amplitudes |
| ... | `N/2 * 4` | `float32[]` | Y positive-frequency amplitudes |
| ... | `N/2 * 4` | `float32[]` | Z positive-frequency amplitudes |

The file length must be exactly `32 + 3 * (N / 2) * 4`. Frequency resolution
is `sampling_rate / N`. The server keeps all `N/2` bins; it does not truncate
the spectrum to 1024 points.

The task UUID is the authoritative identity. The server resolves the sensor and
device from `SensorTask.sn`; the 16-byte header field is informational only.

## Firmware v2 fields needed for reliable physical diagnosis

The next format should add a magic value and version before changing the v1
layout. It should also make these values explicit:

- `timestamp_ms` as `uint64`;
- `bin_count` and `axis_count`;
- actual shaft speed during acquisition (`rpm`);
- window type and amplitude correction convention;
- amplitude unit/scaling (for example peak g, RMS g, or PSD);
- flags describing DC/Nyquist inclusion and clipping/auto-range outcome.

Until a versioned header is agreed, the server continues to accept only the
current v1 layout above and uses the configured device RPM for 1X/2X/3X
diagnosis.
