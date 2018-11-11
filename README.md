# TI_LightCrafter_DLP2000_OSMC_RaspberryPi
a repository to help you configure OSMC on RaspberryPi to use the Texas Instruments DLP 2000 LightCrafter

# DLP Projektor bauen - ständige Displays

- Github: https://github.com/bietiekay/TI_LightCrafter_DLP2000_OSMC_RaspberryPi

- https://www.tindie.com/products/mickmake/pi-projector-rev-2x-series/
- https://www.digikey.com/product-detail/en/texas-instruments/DLPDLCR2000EVM/296-47119-ND/7598640
- Lightcrafter Display on RaspberryPi: http://frederickvandenbosch.be/?p=2948

## schritte OSMC anbindung
- install OSMC from https://osmc.tv
- log in as user "osmc" by SSH
- copy contents of repository to /home/osmc/
- adapt config.txt like boot/config.txt example
- adapt /etc/rc.local like etc/rc.local example
- adapt /etc/modules like etc/modules example
- Notice: check /home/osmc/DLP/dlp_lightcrafter-1.0.19/dlp-lightcrafter/i2c.py -> DEFAULT_I2C_BUS - default is 3, maybe 2 for you
- change to /home/osmc/DLP and run: sudo setup.sh
- reboot

All should work now. I recommend using "Advanced Launcher" from the SRU repo to add shortcuts to brightness.sh to your favorites folder in KODI - this allows you to quickly switch brightness.#

Also there is a /home/osmc/switch script that allows you to toggle between projector and HDMI output if you copied all in boot to /boot.

## Case Measurements
Maße RaspberryPi Zero W (beides - Projektor+MickMake+RaspberryPiZero):
- Hoch: 6cm (57mm)
- Tief: 6cm (55mm)
- Breit: 8cm (78mm)

Maße RaspberryPi 3:
- Pi Platine:
  - Hoch: 2cm (17mm)
  - Tief: 6cm (56mm)
  - Breit: 9cm (85mm)


- Ziel Box RaspberryPi Zero: 60x60x80 oder 70x70x90
  - https://www.reichelt.de/gehaeuse-serie-euromas-ii-82-x-60-x-57-mm-ip65-bopla-em-207-f-p126200.html?&trstct=pol_0
  - Länge: 82mm
  - Breite: 60mm
  - Höhe: 57mm
    - Euromas II - Hochwertige Gehäuseserie aus Polycarbonat
    - Besonders recyclinggerecht durch Verwendung sortenreiner Kunststoffe ohne eingespritzte Metallteile
      - Schutzart IP 65 / DIN EN 60529
      - Farbe hellgrau ähnl. RAL 7035
      - Vertiefte Deckelvariante für die bündige Montage von Folientastaturen
      - Montagefreundliche Verschraubungstechnik
      - Befestigungsnocken für selbstformende Schrauben im Unterteil und im Deckel
      - Brennbarkeit UL94 V2
      - Betriebstemperatur -40/+100 Grad
      - Lieferumfang: Gehäuse, 4 Deckelschrauben V2A

