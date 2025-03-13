# To enable GPS ros node, install NMEA messages:
sudo apt-get install ros-foxy-nmea-msgs

# Install the GPS daemon and run it in the background:

sudo apt install gpsd
gpsd /dev/ttyACM0 -n # assuming this is the USB port for the GPS
pip install gpsd-py3

# Set up gpsd defaults to start on boot:

sudo nano /etc/default/gpsd

# Add these lines below to the file:

START_DAEMON="true"
GPSD_OPTIONS="/dev/ttyACM0"
DEVICES=""
USBAUTO="true"
GPSD_SOCKET="/var/run/gpsd.sock"


# To visualize and use map data, attempting to use nav2:

sudo apt install ros-$ROS_DISTRO-robot-localization
sudo apt install ros-$ROS_DISTRO-mapviz
sudo apt install ros-$ROS_DISTRO-mapviz-plugins
sudo apt install ros-$ROS_DISTRO-tile-map