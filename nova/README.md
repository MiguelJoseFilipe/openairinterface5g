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
