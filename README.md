# Lwip HID Server
## Index
  - [Description](#description)
  - [Started](#started)
  - [Enviorment](#enviorment) 
  - [Client Command](#client-command)
  - [Reference](#reference)
  
## System Preview
![image](https://user-images.githubusercontent.com/108905975/185017083-88404179-c6af-482a-b80c-656b0135556b.png)

## Description
<!--Wirte one paragraph of project description -->  
- This S/W acts as a TCP/IP HID server.  
- Allows the user to directly control the usb-level hardware input, not the SYSTEM API-level, using TCP/IP packets.
- It can be used as a bypass bank security program or game guard.  

## Sequence
![image](https://user-images.githubusercontent.com/108905975/185039260-25943160-a539-488a-9d10-92463f167025.png)
- The two tasks work separately (USB/Lwip)
- If command occur transferring data through the queue
- The semaphore ensures that each task is carried out

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
 - (https://github.com/OpenNuvoton/M480BSP) - BSP
