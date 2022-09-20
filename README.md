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

- boost system (required)
- boost program_options (optional, for example programs)
- cpp-linenoise (optional, for example program ell_interactive. downloaded by cmake)

# how to use
two example programs are provided

## ell_move
which moves a single rotation mount or linear stage connected via one controller.
```
./ell_move -d <controller_tty_path> -m <device_id> -p <position in deg/mm>
```
e.g., to rotate an ELL14K with ID 2 to 90º
```
./ell_move -d /dev/ttyUSB0 -m 2 -p 90
```

## ell_interactive
which provides an interactive prompt that lets you control Elliptec devices connected at a single serial port.

```
./ell_interactive -d <controller_tty_path> -i <list_ _of_ _device_IDs>
```
e.g.
```
./ell_interactive -d /dev/ttyUSB0 -i 0
```