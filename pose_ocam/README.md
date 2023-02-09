# Using sockets for Cpp to Py

Note that `main.cpp` compiles to `./build/ocam`.
As terminal arguments, `ocam` takes the following:

- Drones to send the pose for

```bash
./ocam 1 # represents ID=0
./ocam 2 # represents ID=10
./ocam 3 # represents both IDs
```

- VERBOSE, prints pose estimated and some other details

```bash
./ocam 1 1 # makes verbose to be true
```
