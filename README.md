# Rockchip MPP 

This fork is just to create backup and build/installation instructions.

## Build and Install

Clone and enter repo:
```bash
git clone https://github.com/roman-koshchei/mpp && cd mpp
```

IMPORTANT switch to latest tag instead of develop branch, in my case it's `1.0.11`:
```bash
git checkout tags/1.0.11
```

Run cmake with `/usr` prefix:
```bash
cmake -DCMAKE_INSTALL_PREFIX=/usr -DRKPLATFORM=ON -DHAVE_DRM=ON
```

Run make build:
```bash
make -j$(nproc)
```

Install:
```
sudo make install
```

Add your user to `video` group:
```bash
sudo usermod -aG video $USER
```

Change `/dev/mpp_service` into `video` group by creating file:
```bash
sudo nano /etc/udev/rules.d/99-mpp-service.rules
```

And copying this content to it:
```bash
KERNEL=="mpp_service", MODE="0660", GROUP="video"
KERNEL=="rga", MODE="0660", GROUP="video"
KERNEL=="system", MODE="0666", GROUP="video"
KERNEL=="system-uncached", MODE="0666", GROUP="video"
KERNEL=="system-dma32", MODE="0666", GROUP="video"
KERNEL=="system-uncached-dma32", MODE="0666", GROUP="video"
```

Reload rules to apply changes:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Verify, must be in order: root, video
```bash
ls -l /dev/mpp_service
```

## Original README

Original README of the project is located in `readme.txt` file.
