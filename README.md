# Coyot3pp lib


## SUMMARY

* [INTRODUCTION AND BLAHBLAH](#introduction-blahblah)
* [INSTALLATION](#installation)


## INTRODUCTION BLAHBLAH

This is a set of tools I have created to ease my development tasks.

First of all... as I am doing my tasks for some years, it always arrives the moment when I feel I am writing the same code for different targets.

When I start to write the code, I usually already have traced a *path* of what the code has to explain. Basically it is all about a global task or a project idea that has to be implemented. I use to call it the *high level requirements* (even if I know that in the professional environment it collides with other concepts). These *high level requirements* are implemented using a set of *low level requirements*, and even these *low level requirements* use to need lower level requirements, usually related to the standard c++ implementations.

As time passed, I realized that it was a huge task to my mind to *store* all the *path* to implement the *high level requirements* and, at the same time, *store* the *lower level requirements* and the standard c++ implementations.

As an example, there are many times when I need to *just* obtain the current clock timestamp in milliseconds from epoch. Using standard c++, we can obtain this by:

```cpp
std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
```

And of course you have to have included the libraries... `include <chrono>`... witch happens to be standard c++ since c++11.

For me, having this in mind, at my *RAM* is a weight that, because of my needs and capacities, I do not need to carry. I am the kind of worker that thinks "okey... I know how to do it or to get it, but I do not want to get back every time to remember how to implement it because **I just need to `get_current_timestamp()`**".

And so I realized that *for myself* I needed to create a toolkit that remembered for me all this stuff, the same way as a worker in a garage creates along the years its own toolkits...

This *timestamp* example is a little one. I had many other needs: I needed to relay mqtt communications using just a "*connector*"; I want to be able to have a *QTimer* like tool that does not impose me to include all the `QtCoreApplication` just to ensure that I have a function that is triggered every *x* milliseconds...; If I have to make constants REST requests to a server I want to have a *component* or *agent* that makes it for me and *serves* the result to a callback function, or invokes to another callback if there is some kind of issue with the communications; I want to be able to manage a set of images coming from different sources and be able to manipulate them, so I develop a set of wrappers that fulfills my basic needs (such adding a text to an image... or adding a black-and-white effect) and I use it as an object.

All this library is about that: the way I have constructed a set of components that are built for *my particular way of thinking* (a very simple way of thinking, in fact) that I can reuse along different *projects*, with no need to remember specific implementations of *opencv*, *std::threads*, *chronos*, *jsons*, *files*...

As well, for many times I use it to recall myself some *stuffs* that I learned *time ago*... and it also serves me as a tutorial for myself. For example: the way to create a `cmake` project using `components`...

Finally, I just called it *Coyot3pp* because *ACME* is just very used, *coyot3* is my nick and this library is FAR FAR FAR FAR FAR AWAY to be any kind of standard,...

For all those that may have found this repo, may you find something useful.

## COMPONENTS AND DOCUMENTS

Each component has included its own documentation file:

* [Cor3](./cor3/doc/README.md)
* [R3st](./communication/rest_connector/README.md)
* [Mqtt](./communication/mqtt/README.md)
* [Postgr3Sql](./database/postgresql/README.md)

## INSTALLATION

This library is currently developped for *debian* and derivate distributions. The adaptation for other distributions may require further works with the `cmake` files, and probably, some kind of factorizations.

I have just not tested it with other distributions or with windows. *I did not have the need*.

### DEPENDENCES

at a console, launch the command:

```bash
source INSTALL_DEPS_DEBIAN.sh
```

> this library uses dependences from the standard repositories.

### CHOOSE COMPONENTS

At `CMakeLists.txt`: 

Select the options to activate:

```cmake
option(LCY_BUILD_WITH_MINIMAL_EXAMPLES   "Build with minimal examples"       ON)
option(LCY_BUILD_COMPONENT_REST          "Build with R3st components"        OFF)
option(LCY_BUILD_COMPONENT_MQTT          "Build with Mqtt components"        ON)
option(LCY_BUILD_COMPONENT_POSTGRESQL    "Build with PostgreSql components"  OFF)
option(LCY_BUILD_COMPONENT_QSQLITE       "Build with QSqlite    components"  ON)
option(LCY_BUILD_COMPONENT_IMAG3         "Build with Imag3      components"  OFF)
option(LCY_BUILD_COMPONENT_H264RTSP      "Build with H264Rtsp   components"  ON)
option(LCY_BUILD_COMPONENT_LWSS          "Build with LwsServer  components"  OFF)
```

### BUILD

```bash
mkdir build
cd build
cmake ..
make -j$(nproc)
``` 

### INSTALL

The `CMakeLists.txt` file **forces** the installation at the `install/coyot3pp` directory. In other words: it creates an `install/coyot3pp`.

To install at a specific directory, add the `COYOT3_INSTALL_PREFIX` parameter to cmake.

Example
```bash
...
cmake -DCOYOT3_INSTALL_PREFIX=/usr/local ..
make install
```

**Do not forget:**:

* on success installation, it will be generated an `install_manifest.txt` that contains all the installed files.
* if by any chance you want to install it in `/usr` or `/usr/local`, you will need administrator permissions for the make-install command (such `sudo make install`).
* To clean the installation launch the following command:
  
  ```bash
  cat install_manifest.txt | xargs rm
  ```
  
  However, it has been included a `make uninstall` target that will ease the uninstallation for you.



