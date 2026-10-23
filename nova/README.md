# NOVA non-terrestrial 5G gNB

Deployment of the non-terrestrial 5G cell measured on the NOVA platform: band 66 FDD, 106 PRB at
30 kHz, USRP X310 over 10 GbE, with the satellite Doppler emulation of this branch
(`openair1/SCHED_NR/nr_doppler_shift.c`).

| File | Content |
|---|---|
| `conf/gnb.sa.band66.106prb.usrpx310.ntn.conf` | gNB configuration, including `ntn-Config-r17` for SIB19 |
| `docker-compose.yaml` | gNB container |
| `../tools/ntn_doppler/config.txt` | Doppler emulation: pass, satellite model, delay mode, clock server |
| `../tools/ntn_doppler/TCP_server.py` | experiment clock server |
| `../tools/ntn_doppler/qef_table.csv` | quasi-earth-fixed table (placeholder, device at the beam center) |

## Build the gNB image

From the root of this repository, on branch `nova-ntn`, with the OAI Dockerfiles:

```bash
docker build --target ran-base  --tag ran-base:latest  --file docker/Dockerfile.base.ubuntu .
docker build --target ran-build --tag ran-build:latest --file docker/Dockerfile.build.ubuntu .
docker build --target oai-gnb   --tag oai-gnb:nova-ntn --file docker/Dockerfile.gNB.ubuntu .
```

## Run

The gNB connects to an OAI 5G core (`oai-cn5g-fed`, `docker-compose-basic-nrf.yaml`) on its default
network: AMF at 192.168.70.132, gNB at 192.168.70.129. PLMN 001/01, TAC 1.

```bash
cd nova
python3 ../tools/ntn_doppler/TCP_server.py -v --config ../tools/ntn_doppler/config.txt   # first
docker compose up -d
docker compose logs -f
docker compose down
```

The clock starts when the gNB connects and runs over the visible part of the pass.

## Notes

- `config.txt` selects the satellite model. The earth-moving model is fully defined by the
  configuration. The quasi-earth-fixed model plays back `qef_table.csv`, which must be generated
  for the orbit, beam center and device position of the experiment; the table provided here is a
  placeholder with zero residual Doppler.
- The thread layout in the conf and the container `cpuset` are specific to the host used for the
  measurements (cores of NUMA node 0). Adapt them together.
- The X310 address is its factory default (192.168.30.2).
