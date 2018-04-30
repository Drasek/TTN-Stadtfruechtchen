# TTN-Stadtfruechtchen
Sensorbox for the urban gardening project "Stadtfrüchtchen" in Bonn - https://www.stadtfruechtchen.de/

# Hardware
Heart of the project is the TvB LoRaWAN Node Rev. 2b from Thomas van Bellegem: https://eth0maz.wordpress.com/2017/05/24/upgraded-tvb-lorawan-node-rev-2b/
It got a Atmel 1284p microcontroller together with a RFM95W and solar charge controller on one board.

#Software

This uses OTAA (Over-the-air activation), in the ttn_secrets.h a DevEUI, a AppEUI and a AppKEY is configured, which are used in an over-the-air activation procedure where a DevAddr and session keys are assigned/generated for use with all further communication.

To use this sketch, first register your application and device with the things network, to set or generate an AppEUI, DevEUI and AppKey. Multiple devices can use the same AppEUI, but each device has its own DevEUI and AppKey. Do not forget to adjust the payload decoder function.

# Licence:
GNU Affero General Public License v3.0

# NO WARRANTY OF ANY KIND IS PROVIDED.
