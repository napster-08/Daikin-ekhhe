# Daikin EKHHE esphome external component
This is an esphome external component for the Daikin EKHHE series of hot water domestic heatpumps, also known as the Altherma M HW. 

The component exposes all of the known sensors and settings to the user. 

So far, it's been tested only with an EKHHE260PCV37 model. 

This component has some basic functionality now but still needs to be further developed to be fully functional. 

**!! WARNING !!**: Use at your own risk, has the potential to change the properies of your device and might even damage it if inappropriate settings are applied. 

## YAML setup
In the YAML you can enter which sensors/numbers/selects etc. you want to use. In the example yaml, all available entitiesare listed, you can remove what you don't need. 

When you download the repo, you need to point the yaml to the repository location. Eventually, this repo will be openede up, and a git link can be used. 

Other than this, the only customization available for the module is:
* update_interval

Which sets the interval at which all entities update in seconds. 

If all goes well, you should get something like this in the UI (there are a lot of paramters and variables ...):
![esphome UI example](https://github.com/jcappaert/esphome-daikin-ekhhe/blob/main/images/ekhhe_all.PNG)

## Hardware 
You will need the esp32 UART hooked up to a UART/RS485 converter, connected to A/B/GND in CN23. This will need to be spliced in somehow as the display needs to remain connected. You can tap 5V for an esp32 board from CN21 or CN22. Some information also [here](https://github.com/lorbetzki/Daikin-EKHHE) by lorbetzki. 

## Reverse Engineering
A lot of the protocol reverse engineering has been done by lorbetzki [here](https://github.com/lorbetzki/Daikin-EKHHE).

## TODO
Some main TODOs to get to full functionality are:

* Implement better UART flow control and match the Daikin protocol as indicate by lorbetzki [here](https://github.com/lorbetzki/Daikin-EKHHE/discussions/2#discussioncomment-12176862).  
