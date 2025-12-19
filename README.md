# multi-robot-syclop

## Install Dependencies

```bash
git submodule update --init --recursive
```

```bash
sudo apt install libmsgpack-dev nlohmann-json3-dev
```

## Build

Building for the first time (builds dependencies):
```bash
chmod +x ./build.sh
./build.sh
```

Building after:
```bash
cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="../install" -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF && make -j$(nproc)
```

## Relevant Files

`src/mr_syclop.cpp`: high-level implementation of SyCLoMP and its main function (builds into an executable)

`src/<TBD>.cpp`: comparison methods (Courtney's TODO)
