## dvb-frontends-availink
This repository is the temporary home for Availink demodulator drivers until
they can be upstreamed into media_tree.

Releases are tagged in the format: ```<drivername>_<driverversion>_fw<firmwareversion>```  e.g. avl68x2_2.19.1_fw2.19.27531
### Directory Structure
```
availink/
        |
        \--common/ : Common files used by all Availink drivers.  Includes avl_bsp driver, which is a Linux implementation of the Board Support layer (I2C, wait, mutexes, etc) used by Availink demod SDKs.
        |
        \--avl62x1/ : Linux DVB driver source for AVL62x1 satellite demod family; written atop SDK
        |         |
        |         \--sdk_src/ : AVL SDK for avl62x1 demod;  written atop AVL BSP
        |
        \--avl68x2/ : Linux DVB driver source for AVL68x2 multistandard demod family; written atop SDK
                  |
                  \--sdk_src/ : AVL SDK for avl68x2 demod;  written atop AVL BSP
firmware/ : firmware submodule
```
