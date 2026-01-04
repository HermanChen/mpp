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

## Original README

Original README of the project is located in `readme.txt` file.
