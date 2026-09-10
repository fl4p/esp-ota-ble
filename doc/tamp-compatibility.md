# Tamp wire compatibility

`build_tamp_payload()` selects the legacy tamp format explicitly. Python tamp
2.3.0 defaults to an extended stream (`0x9a` header for this configuration),
which the node's vendored tamp 1.11.1 decoder rejects. `extended=False` emits
the supported `0x98` header. Encoders without that keyword retain their legacy
call path; other compressor errors propagate.

Verified on 2026-09-10: the old payload failed in the real decoder and on the
bench node. The corrected payload reconstructed the exact 757,104-byte image
in native tests (1/17/495/2048-byte fragments) and completed BLE OTA with exact
post-boot image and slot verification. Full-write time was 8.990 s; this is
84.216 kB/s of reconstructed image, 58.885 kB/s of compressed wire data.
These rates must not be described as raw BLE throughput.

## Guard review

1. Empty output, incompatible headers and missing dependencies fail explicitly.
2. Both forbidden low header bits are rejected, individually and together;
   malformed output never becomes a pass through retries.
3. The real payload builder was exercised with installed tamp 2.3.0, through
   the node decoder and the hardware OTA path. API variants have five host tests.
4. Compatibility is checked on the produced bytes, alongside an explicit format
   request. The experiment archive fingerprints encoder/decoder and receiver
   sources; this is not inferred from package version alone.
5. There is no verification cache. A failed compression yields no payload.
6. Only the specific unsupported-keyword TypeError selects the older API;
   internal compressor errors remain errors.
7. The original extended stream is the known-bad control: decoder failure,
   followed by successful legacy decoding and exact image comparison.
8. The fix changes the encoded wire format, so it corrects the incompatibility
   rather than suppressing the receiver's error.

One 757,104-byte sample took 0.212 s to encode with the old default and 0.205 s
with legacy encoding. Output grew from 523,384 to 529,380 bytes. These single
samples establish no encoding-speed improvement; the compatibility check is
one header inspection after compression. Payload-building time is excluded
from the transfer rates above.

Run the API tests with `python3 test/test_tamp_compat.py`; `test/run.sh` also
runs them and propagates failure.
