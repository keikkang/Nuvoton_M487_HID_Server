# Lwip HID Server
## Index
  - [Description](#description) 
  - [Enviorment](#enviorment) 
  - [Client Command](#client-command)
  - [Reference](#reference)
  
## System Preview
![image](https://user-images.githubusercontent.com/108905975/185017083-88404179-c6af-482a-b80c-656b0135556b.png)


## Description
<!--Wirte one paragraph of project description -->  
- This S/W acts as a TCP/IP HID server.  
- Allows the user to directly control the usb-level hardware input, not the software-level, using TCP/IP packets.
- It can be used as a bypass bank security program or game guard.  

## Enviorment
<!-- Write enviromnet about this project -->
- Hardware : Nuvoton-IoT M487 
- Middleware : Lwip, FreeRTOS 
- IDE : ARM Keil uvision 5.36.0.0
- USB : HID (class 1.1)

## Client Command 
 - Client command consists of three parameters
 - 'inputkey' + 'delaytime' + "command dir' 
 - For example "A9_COMMAND" means "A" for "900ms" 
<!--
## Deployment
 Add additional notes about how to deploy this on a live system
 -->
## Reference
<!-- Write the way to contribute -->
 - (https://yhkim4504.tistory.com/2) - Client Side Daemon thread
 - (https://github.com/OpenNuvoton/M480BSP) - BSP
