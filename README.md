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
into `third_party/<os-arch>/{lib,include}/`. At deploy time the bundled
shared libraries sit next to the binary in `bin/lib/`; the binary's `RPATH`
is patched to `$ORIGIN/lib`, so it loads them without touching the host's
package state.

Three build paths, listed in order of how often you'll use them:

| Goal | Command | Output |
| --- | --- | --- |
| Release tarballs for both arches | `make module.tar.gz` | `bin/amd64/module.tar.gz` + `bin/arm64/module.tar.gz` |
| Fast local iteration on host arch | `make setup && make module` | `bin/module.tar.gz` |
| Viam-managed cloud build | `viam module build start` | uploaded by Viam |

### Release build: both arches (`make module.tar.gz`)

This is the build to run before `viam module upload`. It produces
glibc-2.31-portable tarballs for **both** `linux/amd64` and `linux/arm64` from
a single host:

```sh
make module.tar.gz
```

What it does, end-to-end:

1. Builds `third_party/linux-amd64/` and `third_party/linux-arm64/` inside
   an Ubuntu 20.04 container (`UBUNTU_VERSION=20.04`). This pins libfranka
   and Poco to glibc 2.31, the lowest common denominator supported by every
   Ubuntu LTS from 20.04 onward and most modern distros (Debian 11+, RHEL 9+,
   Fedora, etc.).
2. Builds the Go binary inside that same Ubuntu 20.04 container so it picks
   up the matching glibc.
3. Bundles the binary, shared libs, URDF, and STL meshes into a tarball per
   arch.

Outputs:

- `bin/amd64/module.tar.gz`
- `bin/arm64/module.tar.gz`

Prerequisites:

- Docker with `buildx` (or podman with buildx support).
- For cross-arch (e.g. building arm64 from an amd64 host), QEMU binfmt
  emulation must be registered:

  ```sh
  docker run --rm --privileged tonistiigi/binfmt --install all   # docker
  sudo dnf install qemu-user-static                               # podman/Fedora
  sudo apt install qemu-user-static                               # podman/Debian
  ```

Per-arch shortcuts (skip the other arch when you only need one):

```sh
make module-ubuntu20-amd64    # bin/amd64/module.tar.gz only
make module-ubuntu20-arm64    # bin/arm64/module.tar.gz only
```

Note: the per-arch targets do **not** rebuild `third_party/` first. If
you've never built before, or are reproducing a release on a fresh host,
run `make module.tar.gz` (or first run `make third_party-ubuntu20-{amd64,arm64}`
manually) so the vendored libs exist before linking.

#### Uploading to the Viam registry

`make upload` prints the exact commands (it doesn't run them, so you can
review the version string first):

```sh
make upload
# viam module upload --version "0.0.5" --platform "linux/amd64" bin/amd64/module.tar.gz
# viam module upload --version "0.0.5" --platform "linux/arm64" bin/arm64/module.tar.gz
```

Bump the version string in the Makefile's `upload:` target before tagging a
release.

### Local dev build (host arch only)

For tight iteration loops, skip Docker and build natively against the
host's libfranka:

```sh
make setup     # apt-installs deps, builds libfranka 0.9.2 natively
make module    # produces bin/module.tar.gz for the host arch
```

`make setup` is idempotent — it skips the rebuild if `third_party/<triple>/`
is already populated.

Caveat: a natively-built binary is linked against your host's glibc, which
on a recent distro (Fedora 41+, Ubuntu 24.04, etc.) will be too new to load
on a Raspberry Pi or Ubuntu 22.04 deploy host. **Use `make module.tar.gz`
for anything that leaves your machine.**

### Cloud build (Viam's CI pipeline)

[meta.json](meta.json) points Viam's build runners at [setup.sh](setup.sh)
and `make module`. Each runner is already the target arch, so it does a native
build per arch — no docker-in-docker.

```sh
viam module build start
```

### Cleaning up

| Target | Effect |
| --- | --- |
| `make clean` | `rm -rf bin/` |
| `make clean-docker` | prune + remove the `viam-franka-builder` buildx instance and its cache; leaves the default buildx cache (shared with other projects) alone |
| `make distclean` | `clean` + `clean-docker` + wipe `third_party/linux-{amd64,arm64}` and `third_party/build/` |

Reach for `distclean` when you suspect stale vendored libs (e.g. after the
glibc-version error documented in the Makefile comments).

## libfranka version pinning

This module pins **libfranka 0.9.2**, the last release with Panda support.
The libfranka tree at `/home/walicki/Docs/Viam/Accounts/Miele/libfranka` on
the developer's machine is **0.21.2** (FR3-era, pinocchio-based) and is used
only as API reference — the Docker build fetches 0.9.2 from GitHub.

If you later want FR3 support, treat that as a separate model with its own
vendored 0.10+ build, not a version bump of this module.

## Assets: URDF + meshes

[`arm/panda_arm.urdf`](arm/panda_arm.urdf) and the collision STLs under
[`arm/meshes/panda/`](arm/meshes/panda/) are checked into the repo and
bundled into the module tarball by [Makefile:66-71](Makefile#L66). The
URDF's `<collision>` blocks reference the STLs by relative path; Viam's
URDF parser resolves them next to the URDF file and embeds the meshes
into the kinematic model. The arm then renders in the 3D Scene tab.

The STLs were taken from the
[franka_description](https://github.com/frankarobotics/franka_description)
repo's `meshes/robots/fer/collision/` directory (FER is the renamed
Panda). `hand.stl` powers the gripper's 3D rendering via the
`Geometries()` method on the gripper component.

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
