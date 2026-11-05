# NOVA terrestrial 5G gNB

Deployment of the terrestrial 5G cell measured on the NOVA platform: band n78, 273 PRB (100 MHz) at
30 kHz, SISO or 2x2 MIMO, USRP X310 over two 10 GbE links, OAI 5G core.

| File | Content |
|---|---|
| `conf/gnb.sa.band78.273prb.usrpx310.siso.conf` | gNB configuration, SISO |
| `conf/gnb.sa.band78.273prb.usrpx310.2x2.conf` | gNB configuration, 2x2 MIMO |
| `docker-compose.yaml` | gNB container |

## Build the gNB image

From the root of this repository, on branch `nova-fixes`, with the OAI Dockerfiles:

```bash
docker build --target ran-base  --tag ran-base:latest   --file docker/Dockerfile.base.ubuntu .
docker build --target ran-build --tag ran-build:latest  --file docker/Dockerfile.build.ubuntu .
docker build --target oai-gnb   --tag oai-gnb:nova-fixes --file docker/Dockerfile.gNB.ubuntu .
```

## Run

The gNB connects to an OAI 5G core (`oai-cn5g-fed`, `docker-compose-basic-nrf.yaml`) on its default
network: AMF at 192.168.70.132, gNB at 192.168.70.129. PLMN 001/01, TAC 1.

```bash
cd nova
GNB_CONF=gnb.sa.band78.273prb.usrpx310.siso.conf docker compose up -d   # or the 2x2 conf
docker compose logs -f
docker compose down
```

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
