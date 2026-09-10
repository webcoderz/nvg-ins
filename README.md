# nvg-ins

Isolated AGPL-3.0-only inertial-navigation service for NIGHTWATCH. It wraps the pinned public INSLIB `nav_suite` behind a bounded protobuf/nanopb Unix-domain socket. The permissively licensed main application exchanges measurements and navigation state only; no INSLIB source is linked into that process.

## Contract

- Transport: `AF_UNIX`, one connection at a time, `uint32` big-endian payload length followed by one protobuf `nvg.ins.v1.Envelope`.
- Maximum payload: 64 KiB. Maximum IMU batch: 64, strictly increasing device capture times.
- Frames: IMU is FRD, navigation position/velocity is local NED, attitude is Hamilton body-to-NED quaternion in `w,x,y,z` order. GNSS absolute position is WGS84 ECEF metres.
- Covariance: row-major, explicit ordered state names, full 15×15 or 18×18 matrix. State and covariance use the same raw capture time.
- Provenance: protocol/session/clock/origin epochs, mapped monotonic and Unix capture times, timing uncertainty, sequence, and calibration hash are mandatory and echoed.
- Aiding: loose coupling only. `AIDING_MODE_TIGHT` is reserved and never emitted.
- Privacy/security: socket mode `0600`; Linux peer UID must equal the service UID. The supplied unit runs as the pre-provisioned `nvg` user.
- Failure: malformed, nonfinite, out-of-order, oversized, foreign-peer, or unsupported input closes the connection. Reconnect starts an independent estimator. An epoch change resets before consuming the new request.

## Reproducible build

```sh
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Pinned sources:

- INSLIB `3bad57dea66418bae30caa93280b03a889bd7597`
- INSLIB/KFCore `3c3e846f49981dab33d8c69375e958a8feece334`
- nanopb `cad3c18ef15a663e30e3e43e3a752b66378adec1` (0.4.9.1)
- Protocol compiler for checked-in bindings: protobuf `v21.12`

Generated `ins.pb.c/.h` is checked in so the target build does not download or execute a generator. Regenerate it only with nanopb 0.4.9.1 and the checked-in `protocol/ins.options` bounds.

## Run

The service defaults to a private socket and a bounded 100 Hz output cadence:

```sh
nvg-ins --socket /run/nvg/ins.sock --output-hz 100
```

CPU affinity and FIFO scheduling are explicit deployment inputs. Use them only on
a core reserved by the target profile; unsupported affinity or scheduler requests
fail startup rather than silently degrading:

```sh
nvg-ins --socket /run/nvg/ins.sock --output-hz 100 --cpu 6 --sched fifo --priority 20
```

Offline qualification uses the same validation, estimator and protobuf framing path.
Both files are concatenated big-endian-length-prefixed `Envelope` streams:

```sh
nvg-ins --replay requests.pbstream --output states.pbstream
```

`covariance-oracle` independently feeds the pinned upstream C API, obtains
`ins_suite_get_covariance`, and compares every element against the service's
row-major UDU reconstruction. `socket-smoke` exercises the actual private UDS
service and is intentionally launched by integration tooling rather than CTest.

## Legal boundary

This repository and the resulting `nvg-ins` executable are AGPL-3.0-only because they link INSLIB. The main NIGHTWATCH application is a separate process and communicates only through the published wire contract. Keep deployment source-offer and corresponding-source obligations with this service package. This boundary is an engineering packaging choice, not legal advice.

## Qualification status

The host replay proves framing, validation, reset, covariance serialization, and deterministic adapter behavior. It does not qualify a physical IMU, GNSS receiver, pressure sensor, magnetometer, Jetson timing, calibration, drift, thermal behavior, or navigation accuracy.
