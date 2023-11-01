Currently supported clients:
- `debug`
- `ivy`

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
