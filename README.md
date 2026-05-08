# viam-franka-arm

Viam module for the **Franka Emika Panda** arm and **Franka Hand** gripper.
Wraps [libfranka](https://github.com/frankaemika/libfranka) (C++) via a thin
cgo C-ABI shim. The 1 kHz FCI control loop runs entirely inside libfranka in
C++; Go calls high-level move primitives and polls state at a lower rate.

## Status

Phase 1 scaffolding. Not yet connected to real hardware.

## Target architectures

- `linux/arm64` — primary
- `linux/amd64`, `darwin/arm64` — planned

## Prerequisites on the deployment host

- A real-time-patched Linux kernel (`PREEMPT_RT`). The Franka Control
  Interface drops the connection if the 1 kHz loop misses deadlines.
- The robot reachable over Ethernet (default `172.16.0.2`).
- The arm in FCI mode via the Desk web UI, with the e-stop released.

## Build

The module statically vendors `libfranka` and its dependencies (Eigen, Poco)
into `third_party/<os-arch>/{lib,include}/`. Vendoring is done via a Docker
buildx recipe so the build host doesn't need apt-get magic.

```sh
# One time per arch:
make third_party-arm64

# Then:
make module
# -> bin/module.tar.gz
```

`make build` patchelf's the Go binary's rpath to `$ORIGIN/lib`, so the bundled
`.so`s are loaded from next to the binary at runtime regardless of where Viam
unpacks the tarball.

## libfranka version pinning

This module pins **libfranka 0.9.2**, the last release with Panda support.
The libfranka tree at `/home/walicki/Docs/Viam/Accounts/Miele/libfranka` on
the developer's machine is **0.21.2** (FR3-era, pinocchio-based) and is used
only as API reference — the Docker build fetches 0.9.2 from GitHub.

If you later want FR3 support, treat that as a separate model with its own
vendored 0.10+ build, not a version bump of this module.

## Asset: Panda URDF

Drop a `panda_arm.urdf` into `arm/` before building. Source it from the ROS
[`franka_description`](https://github.com/frankaemika/franka_description)
package. Viam's motion planning consumes the URDF through `ModelFrame`.

## Configure your Panda

```json
{
  "name": "arm-1",
  "model": "viam:franka:panda",
  "type": "arm",
  "attributes": {
    "host": "172.16.0.2",
    "speed_factor": 0.1
  }
}
```

## Franka Hand

```json
{
  "name": "gripper-1",
  "model": "viam:franka:gripper",
  "type": "gripper",
  "attributes": {
    "host": "172.16.0.2"
  }
}
```
