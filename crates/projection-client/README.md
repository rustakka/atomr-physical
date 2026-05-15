# atomr-physical-projection-client

`atomr-projection-client` is the remote-node companion to the
`atomr-physical-projection` crate. It runs on a small ARM device
(Raspberry Pi, Jetson Nano, etc.) that acts as a Moonlight screen for
an atomr-physical workstation on the same LAN.

## Purpose

The workstation's `ProjectionActor` spawns one `sunshine` server per
virtual display and advertises each as an `_nvstream._tcp.local.`
service. This client discovers those services over mDNS, completes
Sunshine's self-signed HTTPS pairing handshake (announce + PIN), then
execs `moonlight-embedded` so the operator sees the video session
inline in the controlling terminal. It deliberately keeps no
persistent state: every invocation is a fresh pair-and-stream cycle,
which keeps the deployment surface on the remote node down to a single
static binary and one systemd unit.

## Subcommands

### `discover`

Browse the LAN for matching Sunshine instances and print them.

```
atomr-projection-client discover \
    --service-filter 'atomr-.*' \
    --timeout-secs 5
```

Each matching service is printed on its own line as
`<instance> <host>:<port> <txt-summary>`.

### `run`

Pair against the first matching service and exec `moonlight-embedded`.

```
atomr-projection-client run \
    --service-filter 'atomr-.*' \
    --moonlight-bin /usr/bin/moonlight-embedded
```

Useful flags:

- `--client-id <ClientId>` — reuse a previously paired client identity
  instead of generating a fresh `cli-<uuid>`.
- `--hostname <name>` — override the friendly name sent in the pairing
  payload. Defaults to `$HOSTNAME`.
- `--manual-pin` — print the PIN to stdout and wait on stdin so an
  operator can type the PIN into the server-side CLI before pairing
  proceeds.
- `--dry-run` — pair but skip the `moonlight-embedded` exec (CI).

## Cross-compile for Raspberry Pi / Jetson

```
rustup target add aarch64-unknown-linux-gnu
cargo build --release \
    --target aarch64-unknown-linux-gnu \
    -p atomr-physical-projection-client
```

You'll need an `aarch64` linker installed on the host
(`aarch64-linux-gnu-gcc` from the `gcc-aarch64-linux-gnu` package on
Debian/Ubuntu) and the matching env var pointing at it, for example:

```
export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc
```

The output binary lives at
`target/aarch64-unknown-linux-gnu/release/atomr-projection-client`.

## Systemd install

On the remote node, copy the binary and the bundled unit into place,
then enable it:

```
sudo install -m 0755 \
    target/aarch64-unknown-linux-gnu/release/atomr-projection-client \
    /usr/local/bin/

sudo install -m 0644 \
    contrib/atomr-projection-client.service \
    /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable --now atomr-projection-client.service
```

Edit the unit's `User=`/`Group=` and `ExecStart=` flags to match the
deployment (the shipped unit assumes a Raspberry Pi default user).
Logs land in the journal: `journalctl -u atomr-projection-client`.

## Security note

Sunshine ships a self-signed TLS certificate per instance, so this
client uses `danger_accept_invalid_certs(true)` on the reqwest client
that talks to `/api/pair` and `/api/pin`. That is acceptable for a
trusted LAN where the mDNS namespace and the physical network are both
under the operator's control. Deployments that need stronger
guarantees should pre-distribute each instance's certificate
out-of-band and pin it on the client; the current binary does not
support that mode.
