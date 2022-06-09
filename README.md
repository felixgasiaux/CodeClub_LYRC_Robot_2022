# CodeClub_LYRC_Robot_2022
This is the repo for documenting our robot build of the 2022 LYRC. 

## Team members are:
* Noé Guerien
* Kim Bourg
* Avanti Sharma 
* Félix Gasiaux 

The entire robot is built in [Level 2](https://level2.lu) in Bonnevoie.
This Team is formed from members of the [CodeClub](https://codeclub.lu) association in Luxembourg and mentored by Thierry Goniva, Jean-Phillipe Guisset and Miguel (I do not know the name sorry).

## The Robot
The kit consists of the following components:
* Makeblock Ultimate 2.0 kit
* Makeblock smartcam (Pixie)
* Makeblock color sensor
* up to 150g of passive enhancement(s)


## Install
You have to install the firmware for the Makeblock Robot from their [repository](https://github.com/Makeblock-official/Makeblock-Libraries) . This is also where you can find the Firmwae for the different rovers. Just import it trough the normal Arduino IDE and it should work. 
We add the camera on port 6 for now and copy the library from the hidden apllication folder from the [Makeblock App](https://www.makeblock.com/software/)  for PC/Mac to the folder where the Arduino libraries are stored(Documents/Arduino/libraries).
You have to add the Camera library to where you install the normal libraries. This library allows you to use the camera with the mbot. 
