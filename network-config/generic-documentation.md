# Generic Documenetation about the Network Setup


### SAMI NET
is a robust, simple and easily configurable network dedicated to providing a reliable communications link between the rover and the control station (also refered to as the base station). The network is able to function over long distances relaibly with some near line of sight functionality with the use of 2 high-gain antennas over the 2.4GHz frequencies (Unliscenced). On the rover side an omni-directional antenna **{insert model}** is powered by an access point (Ubiquiti Rocket M2). On the base station side there is a directional antenna **{insert model}** that transmits radio frequencies within a range of 120 degrees. 

### Configuration (Network Layer)
The network is easily managed through Ubiquiti's network management portals open by default at the adrress: 192.168.1.20, this applies to both of the access points. Although the base station antenna is using a more sophisticated access poing (Prism 2AC) allowing for easy access using your phone by connecting to it's management radio through an app on your phone. The topography diagram (figure 1.0) shows the configuration and assignment of addresses for each of the access points as well as the network devices applicable to our rover setup.
![DrawioNetowrkSchema drawio](https://github.com/CPRT/2023Rover/assets/34238398/9e57df7f-cc05-4b09-9e32-390e998873df)


### Initial Setup
