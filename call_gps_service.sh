


echo "GPS coords to a spot in the grass just outside the bay"
ros2 service call /commander/nav_to_gps_geopose interfaces/srv/NavToGPSGeopose '{ goal: { position: { latitude: 45.3852279, longitude: -75.6983036, altitude: NaN}}}'
