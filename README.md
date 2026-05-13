# viam-franka-arm

Viam module for the **Franka Emika Panda** arm and **Franka Hand** gripper.
Wraps [libfranka](https://github.com/frankaemika/libfranka) (C++) via a thin
cgo C-ABI shim. The 1 kHz FCI control loop runs entirely inside libfranka in
C++; Go calls high-level move primitives and polls state at a lower rate.

## Status

Phase 1 scaffolding. Not yet connected to real hardware.

## Target architectures

- `linux/amd64` — implemented, primary deployment target
- `linux/arm64` — implemented
- `darwin/arm64` — planned (developer convenience only; FCI requires RT Linux)

## Prerequisites on the deployment host

- A real-time-patched Linux kernel (`PREEMPT_RT`). The Franka Control
  Interface drops the connection if the 1 kHz loop misses deadlines. On
  Ubuntu 26.04 LTS, install with `sudo apt install ubuntu-realtime` and
  reboot into the RT kernel. Confirm with `uname -v | grep -i PREEMPT_RT`.
- The user running `viam-agent` must be allowed real-time scheduling. Add
  to `/etc/security/limits.conf` (or a drop-in under `limits.d/`):

  ```conf
  @realtime soft rtprio 99
  @realtime soft priority 99
  @realtime soft memlock unlimited
  @realtime hard rtprio 99
  @realtime hard priority 99
  @realtime hard memlock unlimited
  ```

  Create the group if it doesn't exist and add the `viam` user to it:

  ```sh
  sudo groupadd -r realtime
  sudo usermod -aG realtime viam
  ```

  No reboot is needed — `limits.conf` is read by `pam_limits.so` at login
  time, so the new limits take effect on the next login session for `viam`
  (log out and back in, or open a fresh SSH session). Verify with
  `ulimit -r -l` as the `viam` user.

- **Note that `viam-agent` is run as a systemd service**, `limits.conf` does NOT
  apply — systemd units skip PAM session setup. Set the limits on the unit
  instead via a drop-in:

  ```sh
  sudo systemctl edit viam-agent
  ```

  ```ini
  [Service]
  LimitRTPRIO=99
  LimitNICE=-20
  LimitMEMLOCK=infinity
  ```

  Then `sudo systemctl daemon-reload && sudo systemctl restart viam-agent`.
  Verify with `systemctl show viam-agent | grep -E 'LimitRT|LimitMEM'`.
- The robot reachable over Ethernet (default `172.16.0.2`).
- The arm in FCI mode via the Desk web UI, with the e-stop released.

### Runtime libraries

**Nothing libfranka specific needs to be installed on the deployment host.**
The module tarball bundles `libfranka.so`, `libfrankamodel*.so`, the Poco
shared libraries (`PocoFoundation`, `PocoNet`, `PocoUtil`, `PocoXML`,
`PocoJSON`) and `libpcre.so.3` under `bin/lib/`. The Go binary's `RPATH`
is patched to `$ORIGIN/lib`, so it loads them from next to itself —
independent of the host's package state.

If you ever see `libPocoNet.so.80: not found` at startup, the rpath
patch didn't take — re-run `make module` rather than `apt install
libpoco-dev`, since installing system Poco 90+ on a 26.04 host won't
provide the `.so.80` ABI libfranka 0.9.2 was linked against.

## Build

The module statically vendors `libfranka` and its dependencies (Eigen, Poco)
into `third_party/<os-arch>/{lib,include}/`.

### Cloud build (Viam's build pipeline)

[meta.json](meta.json) declares a `setup` step that runs [setup.sh](setup.sh)
inside Viam's per-arch build runner. The runner is already the target arch,
so `setup.sh` does a native cmake build of libfranka 0.9.2 — no
cross-compilation, no docker-in-docker.

```sh
# After registering the module on Viam:
viam module build start
```

### Local cross-arch build (developer convenience)

For local builds when your host arch differs from the target, `make
third_party-arm64` uses Docker/podman buildx to vendor libfranka:

```sh
make third_party-arm64    # one-time per arch, produces third_party/linux-arm64/
make module               # produces bin/module.tar.gz
```

### Local native build

If your host *is* the target arch, run `setup.sh` directly:

```sh
make setup                # apt-installs deps, builds libfranka natively
make module
```

`make build` patchelf's the Go binary's rpath to `$ORIGIN/lib`, so the bundled
`.so`s load from next to the binary at runtime regardless of where Viam
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
