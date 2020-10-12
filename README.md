# Adaptive Foraging in Dynamic Environments Using Scale-Free Interaction Networks
This repository contains the development sources that can be used to reproduce the simulations from the paper "Adaptive Foraging in Dynamic Environments Using Scale-Free Interaction Networks".

Specifically, the repository provides a C++ simulation of the collective foraging behaviour with scale-free communication and dynamic changes of item avaialability using ARGoS.

Please visit https://osf.io/48b9h for more information about the project, to get the exact source files that were used for the experiments as well as more detailed elaborations on the data analysis procedures.

You can find our corresponding paper here https://doi.org/10.3389/frobt.2020.00086

If our projects sparks your interest in swarm robotic simulation, please visit the ARGoS webpage for more instructions on how to easily get started. http://argos-sim.info/

Our simulation setup is strongly inspired by the ARGoS foraging example http://argos-sim.info/examples.php

However, we significantly extended the scenario by scaling up the swarm to almost 1000 robots. Moreover, we included an algorithm that allows the swarm to grow a scale-free network based on proximity information at each simulation time step. Additionally, we implemented a time-varying availability of food items inside the foraging area.

If you wish to run these simulations on your computer to reproduce our results, or just for fun, you can use this repository or go to https://osf.io/48b9h/ where you find everything you need - including a short tutorial on how to install the simulator (e.g. using our Docker image) and get started with ARGoS.   
_____________

## Installation
The following instructions are adopted from https://osf.io/48b9h/wiki/Installation/

### Step zero: remove installed versions
- If you have previously installed versions of ARGoS, it is recommended to remove them first.      
You can find a short guide on how to remove ARGoS here: https://osf.io/n7kr3/wiki/Compilation%20and%20Installation/     
In a nutshell, use the following commands to remove ARGoS from your system's shared libraries:
```
#-- On Linux/Ubuntu/Debian
sudo dpkg --remove argos3-simulator
# If on your Linux/Ubuntu/Debian system ARGoS was previously installed not through dpkg
sudo rm -vr /usr/local/*/argos3*
# On MacOS
sudo brew remove argos3
```
And remove any local configurations associated with ARGoS:
```
# On Linux
rm $HOME/.config/ARGoS/ARGoS.conf
# On MacOS
defaults write info.argos-sim.ARGoS
```

### Step one: download and install dependencies
Please follow [these instructions](https://github.com/ilpincy/argos3#requirements) to install all required packages on your system.   
At the time of writing these instructions, these are the packages you need (please make sure you have approx. 1.5 GB of free memory):   
```
# On Linux/Ubuntu/Debian
sudo apt-get update
sudo apt-get install cmake libfreeimage-dev libfreeimageplus-dev \
  qt5-default freeglut3-dev libxi-dev libxmu-dev liblua5.3-dev \
  lua5.3 doxygen graphviz graphviz-dev asciidoc
  
# On Mac, please use HomeBrew
brew install pkg-config cmake libpng freeimage lua qt \
  docbook asciidoc graphviz doxygen
```

In case of errors, it sometimes helps to configure the locale settings:
```
sudo localedef -i en_US -f UTF-8 en_US.UTF-8;
export LANGUAGE=en_US.UTF-8;
export LANG=en_US.UTF-8;
export LC_ALL="en_US.UTF-8"
export LC_CTYPE="en_US.UTF-8"
sudo locale-gen en_US.UTF-8;
```
### Step two: download and install from source
To install ARGoS, please download `argos3-v_for_dyn_042020.zip` from https://osf.io/48b9h/files/.     
Then, extract the files, compile them and install ARGoS.
```
unzip argos3-14dadfd.zip -d argos3
cd argos3
mkdir build
cd build
cmake -DARGOS_DOCUMENTATION=OFF -DCMAKE_BUILD_TYPE=Release ../src
make
sudo make install
```
(where *14dadfd* is the id of the commit in the [ARGoS repository on github](https://github.com/ilpincy/argos3)).     

### Step three: download and compile `collective-foraging-dynenv`
`collective-foraging-dynenv` is this project's working directory. This code can be used to control the experiment, adjust the parameters, perform the data analysis, etc.    
Download the development sources:
```
git clone https://github.com/iljaRa/collective-foraging-dynenv.git
```
Descned into the `Argos-experiments/` directory and compile the code by executing:
```
cd collective-foraging-dynenv/Argos-experiments
rm -r build
mkdir build
cd build
cmake ..
make
cd ..
```

You can test whether everything was successfully installed by running the following line and comparing the outcome to https://osf.io/48b9h/wiki/Running%20simulations/:
```
argos3 -c experiments/foraging_large_with_visualization.argos
```

## References
Rausch, I., Khaluf, Y., & Simoens, P. (2020, May 4). Adaptive Foraging in Dynamic Environments Using Scale-Free Interaction Networks. Retrieved from osf.io/48b9h 
