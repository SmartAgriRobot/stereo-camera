# Stereo Camera
a type of camera with two or more lenses with a separate image sensor or film frame for each lens. This allows the camera to simulate human binocular vision, and therefore gives it the ability to capture three-dimensional images

## Features
* Live depth image

## Installation
```
sudo apt update
sudo apt install python3-pip python3-gpiozero python3-serial \
	python3-opencv python3-numpy python3-matplotlib python3-sklearn \
	python3-sklearn-lib libx264-dev libjpeg-dev libgstreamer1.0-dev \
	libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev \
	gstreamer1.0-plugins-ugly gstreamer1.0-tools gstreamer1.0-gl gstreamer1.0-gtk3
```
## Compatibility
### Hardware :
* Raspberry Pi Zero 2 W
* ESP32-CAM

### Software :
* Raspberry Pi OS (Bullseye)
* Python 3

## Usage
```
git clone https://github.com/SmartAgriRobot/stereo-camera.git
cd stereo-camera
cd sourcecode
python3 main.py
```
## Video Guide
[SmartAgriRobot Channel](https://www.youtube.com/channel/UCOgiOXJ43hnMZIsxGAZKoPQ)
## License and Credits
The Stereo Camera is released under the MIT license.

## Disclaimer
This source and the whole package comes without warranty. It may or may not harm your computer or cell phone. Please use with care. Any damage cannot be related back to the author. The source has been tested on a virtual environment and scanned for viruses and has passed all tests.
