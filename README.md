# Lwip HID Server
## Index
  - [Description](#description) 
  - [Enviorment](#enviorment) 
  - [Client Command](#client-command)
  - [Reference](#reference)

## Description
<!--Wirte one paragraph of project description -->  
- This S/W acts as a TCP/IP HID server.  
- Allows the user to directly control the usb-level hardware input, not the software-level, using TCP/IP packets.
- It can be used as a bank security program or game guard bypass macro.  

## Enviorment
<!-- Write enviromnet about this project -->
- Hardware : Nuvoton-IoT M487 
- Middleware : Lwip, FreeRTOS 
- IDE : ARM Keil uvision 5.36.0.0
- USB : HID Class(1.1ver)

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
 - [Client](https://yhkim4504.tistory.com/2) #Using Python
 - [Server](https://github.com/OpenNuvoton/M480BSP) #BSP
