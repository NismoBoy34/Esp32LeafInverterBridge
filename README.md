# Nissan-Leaf-Esp32-Can-Bridge 
# Author: Adam Saiyad (adamsaiyad@gmail.com) , Julius Calzada (julius.jai@gmail.com)
An Arduino IDE based migration to Esp32 based off Muxsan/Dala MicroChip atmel can Bridge https://github.com/dalathegreat/Nissan-env200-Battery-Upgrade
Due to the Chip shortage and shortages of other components this library was created to offer a more accessible and attainable method to allow the can bridge to be produced .
It has been tested thorougly in the environment of a vehicle using an Esp32 Devkit and a made up mcp2525 board .
The Build includes a method of changing between EN200 and Nissan Leaf functions easily from the config.h file . (please ensure to select the correct vehicle before compiling )

Things to be done and not implemented yet .
1. Can bridge Web Server to allow changes to be made (via Hotspot ) on the esp32
2. add libraries for inverter upgrade and charging limitation for example 50% 80% etc as well as hyper miling .
3. Add frame work for webserver Menus and design layout something simple and easy to work with 
4. Over the air updates for easy upgradeable software versions .

Not tested 
1. The pcb design schematic is still in proofing we need to assemble and test this .
2. connectors need to be tested and possibly changed if they are not easy to get .
3. hardware power supply from 12v not tested yet 
4. Upload using jumpers on the PCB ( to be tested and verified )

