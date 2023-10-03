# Development in DevContainer


You can easily try this example with `DevContainer` in `VSCode`, without installing any packages on your host system. 

Depending on the system setup, you may need to do the following. 

To use an Nvidia GPU, add the following variables to `.devcontainer/docker-compose.yml`
```yaml
environment:
    DISPLAY: $DISPLAY
    NVIDIA_VISIBLE_DEVICES: all
    NVIDIA_DRIVER_CAPABILITIES: all
runtime: nvidia
```
Otherwise, comment it out.


Allow the root user to connect to X server:
```bash
xhost + local:root
```
