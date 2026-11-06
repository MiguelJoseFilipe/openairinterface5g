# NOVA terrestrial 5G gNB

Deployment of the terrestrial 5G cells of the NOVA platform: band n78 at 30 kHz, SISO or 2x2 MIMO, USRP
X310, OAI 5G core. The 273 PRB cell is the one measured. An OAI nrUE can attach to the 106 and 51 PRB cells.

| File | Content |
|---|---|
| `conf/gnb.sa.band78.<273,106,51>prb.usrpx310.<siso,2x2>.conf` | gNB configurations |
| `conf/nrue.band78.<106,51>prb.env` | nrUE settings of each cell |
| `conf/nrue.uicc.conf` | example nrUE SIM, with the keys of the OAI test subscriber |
| `docker-compose.yaml` | gNB container |
| `docker-compose.ue.yaml` | nrUE container |

## Radio link and cell

The gNB and the nrUE run with `-E` (three-quarter sampling). The rate is 4 bytes per sample and per channel,
before packet overhead (sample rates from `common/utils/nr/nr_common.c`):

| PRB | MSps | Rate per channel | Over one 1 GbE link | Over one 10 GbE link |
|---|---|---|---|---|
| 273 | 184.32 | 5.9 Gb/s | no | SISO only; 2x2 needs two links |
| 106 | 46.08 | 1.47 Gb/s | no | SISO and 2x2 |
| 51 | 23.04 | 737 Mb/s | SISO only | SISO and 2x2 |

A 1 GbE link to an X310 needs the HG FPGA image and an SFP in port 0, at address 192.168.10.2.

## Build the gNB image

From the root of this repository, on branch `nova-fixes`, with the OAI Dockerfiles:

```bash
docker build --target ran-base  --tag ran-base:latest   --file docker/Dockerfile.base.ubuntu .
docker build --target ran-build --tag ran-build:latest  --file docker/Dockerfile.build.ubuntu .
docker build --target oai-gnb   --tag oai-gnb:nova-fixes --file docker/Dockerfile.gNB.ubuntu .
```

For the nrUE, after the first two lines:

```bash
docker build --target oai-nr-ue --tag oai-nr-ue:nova-fixes --file docker/Dockerfile.nrUE.ubuntu .
```

Build from a clone with its submodules (`git clone --recurse-submodules`). In a git worktree, or without the
submodules, the configuration stops at `openair2/E2AP/CMakeLists.txt`.

## Run

The gNB connects to an OAI 5G core (`oai-cn5g-fed`, `docker-compose-basic-nrf.yaml`) on its default
network: AMF at 192.168.70.132, gNB at 192.168.70.129. PLMN 001/01, TAC 1.

```bash
cd nova
GNB_CONF=gnb.sa.band78.273prb.usrpx310.siso.conf docker compose up -d   # or the 2x2 conf
docker compose logs -f
docker compose down
```

`GNB_SDR_ADDRS` replaces the X310 addresses, for instance `addr=192.168.40.2` on a single link. `GNB_IMAGE`
replaces the image, and `GNB_EXTRA_OPTIONS` adds options.

## Run the nrUE

The nrUE needs a B210 over USB 3 (default) or an X310 (`UE_USRP_ARGS`). Declare its SIM in the subscriber
database of the core; `UE_UICC` replaces the example file.

```bash
cd nova
docker compose -f docker-compose.ue.yaml --env-file conf/nrue.band78.106prb.env up -d
```

`UE_CPUSET` restricts the container to a list of cores. To test a cell without radio, on one host:

```bash
GNB_EXTRA_OPTIONS="--rfsim --rfsimulator.[0].serveraddr server" docker compose up -d
UE_EXTRA_OPTIONS="--rfsim --rfsimulator.[0].serveraddr 127.0.0.1" \
  docker compose -f docker-compose.ue.yaml --env-file conf/nrue.band78.106prb.env up -d
```

For a 2x2 cell, add `--ue-nb-ant-tx 2 --ue-nb-ant-rx 2` to `UE_EXTRA_OPTIONS`.

## Notes

- The gNB runs with `-E` (three-quarter sampling): 273 PRB is then sampled at 184.32 MHz, which the
  X310 master clock supports.
- The thread layout in the confs (`thread-pool`, `L1_rx_thread_core`, `L1_tx_thread_core`, RU `tp_cores`)
  and the container `cpuset` are specific to the host used for the measurements (cores of NUMA node 0).
  Adapt them together.
- The X310 addresses are its factory defaults (192.168.30.2 and 192.168.40.2).

## Provenance of the measurements

The run manifests of the NOVA 5G database identify the gNB configuration by its file name
(`conf_path`) and its SHA-256 (`conf_sha256`):

| Configuration | Name in the manifests | `conf_sha256` |
|---|---|---|
| `gnb.sa.band78.273prb.usrpx310.2x2.conf` | `00101_UTD_273PRB_2x2.conf` | `957426cf0b4db4488bd1480604d68f39430662969f8a38e08969f1820979783f` |
| `gnb.sa.band78.273prb.usrpx310.siso.conf` | `00101_UTD_273PRB_SISO.conf` | `9c69c90274448ddf8872d038b2f66fde234caf103974499452c7ce4b2944cdaa` |

The hashes are those of the files used on the measurement host. The files here set every parameter
to the same value. They differ only in their comments and in `sdr_addrs`, from which the X310
management address on the laboratory network (`mgmt_addr`) is removed. Their hashes therefore differ
from the recorded ones.
