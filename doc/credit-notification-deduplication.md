# Do not flood the final credit grant

After credit reached the payload's full size, `grantCredit()` emitted that
same value on every drain. The node's three intermediate raw OTA runs sent
379–407 duplicate final grants each. Only advancing grants should be emitted
immediately; the existing five-second repeat handles a lost notification.

The fix requires `g > lastGranted` before either immediate-grant condition.
The initial grant and every credit advance remain intact. On the benchmark's
754,624-byte raw transfer, the candidate sent 29 credit messages, including
exactly one final grant. The image was fully programmed and its exact identity
and changed running slot verified after boot. The first two candidate runs
measured 66.67 and 69.56 kB/s; this is not evidence of a large isolated speedup.

## Review of the condition

1. It makes no new success verdict: malformed commands, bad image digests,
   short transfers and flash failures still fail through the receiver's checks.
2. An unchanged or decreasing value cannot become an immediate new grant.
   Valid advances remain bounded by the payload size.
3. The regression exercises the actual receiver: initial credit, 32 drains,
   the interval before five seconds, timed recovery and successful completion.
4. The source condition and regression are committed together; hardware source,
   image identity and logs are retained in the throughput experiment archive.
5. There is no cached verification. `lastGranted` remains the existing credit
   high-water mark, reset for a new transfer.
6. Recovery still emits the existing grant after five seconds. It is deliberately
   distinct from an immediate advance; neither implies image verification.
7. The original code produces 33 identical grants in the 32-drain control and
   fails the regression. The candidate produces one, then the timed repeat.
8. This removes redundant protocol traffic; it does not remove an error report
   or relax the receiver's bounds, digest check or final flash validation.

A synthetic optimized native-host benchmark over 10 million evaluations
measured 0.85 ns/check for the new condition and 2.75 ns/check for the old one.
That is not an ESP32 timing estimate or a reliable claim of CPU speedup; there
is no new allocation or I/O, and the measured traffic reduction is the useful
effect. All three receiver erase-strategy test suites cover the regression.
