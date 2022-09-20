# how to build

```
git clone https://gitlab.com/p.d/elliptecpp.git
cd elliptecpp
mkdir build
cd build
cmake ..
make
sudo make install
```

# dependencies

- cpp-linenoise 
- boost program_options

# how to use
two example programs are provided

## ell_rot
which moves a single connected

## ell_interactive
which provides an interactive prompt that lets you control Elliptec devices connected at a single serial port.

```
./ell_interactive -d <controller_tty_path> -i <list_ _of_ _device_IDs>
```
e.g.
```
./ell_interactive -d /dev/ttyUSB0 -i 0
```