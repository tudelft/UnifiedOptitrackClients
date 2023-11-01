Currently supported clients:
- `debug`
- `ivy`
- `udp`

Build all with:
```shell
mkdir build && cd build
cmake .. && make
```

Build only some with (for example):
```shell
mkdir build && cd build
cmake -D'CLIENTS=debug' .. && make
```

## Prerequisites

TODO: how to get ivy-c-dev?

