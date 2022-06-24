# CodeClub_LYRC_Robot_2022
This is the repo for documenting our robot build of the 2022 LYRC. 

## Index

  1. [Our Team](https://github.com/felixgasiaux/CodeClub_LYRC_Robot_2022#about-the-team)
  2. [The hardware used](https://github.com/felixgasiaux/CodeClub_LYRC_Robot_2022#hardware)
  3. [Installation](https://github.com/felixgasiaux/CodeClub_LYRC_Robot_2022.git#installation)

---

## About the Team
* Noé Guerien
* [Kim Bourg](https://github.com/B0urg)
* [Avanti Sharma](https://github.com/SelfieQueen101)
* [Félix Gasiaux](https://github.com/felixgasiaux)

The entire robot is built in [Level 2](https://level2.lu) in Bonnevoie.
This Team is formed from members of the [CodeClub](https://codeclub.lu) association in Luxembourg and mentored by Thierry Goniva, Jean-Phillipe Guisset and Miguel (I do not know the name sorry).

---

## Hardware
The kit consists of the following components:
* [Makeblock Ultimate 2.0 kit](https://www.makeblock.com/steam-kits/mbot-ultimate)
* [Makeblock smartcam (Pixie)](https://education.makeblock.com/smart-camera/)
* [Makeblock color sensor](https://www.makeblock.com/project/me-color-sensor-v1)
* up to 150g of passive enhancement(s)
* [Makeblock MeRGB LED](https://www.makeblock.com/project/me-rgb-led)
---

## Installation

To use our Code you first need to clone our Repository :
```bash
git clone https://github.com/felixgasiaux/CodeClub_LYRC_Robot_2022.git
```
Then you will need to include the libraries into your arduino IDE :
 > Under Sketch > Include Library > Add .ZIP Library 

 > Then select the Library you just downloaded with in the repository

Next you need to make sure that you have selected the right Board & Port :

 > Go to Tools > Board: ... and select Arduino Mega or Mega 2560

 > Go to Tools > Port and select your Port

 > Connect the Sensors to the correct Ports on the MegaPie
 
Now open the .ino File from our Firmware, upload the code and have fun!
