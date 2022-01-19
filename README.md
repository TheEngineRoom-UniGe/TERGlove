# TERGlove

This repository contains the code needed to use the TER data glove.
To upload the firmware on the TER glove use the Arduino ide. To properly configure the Arduino IDE for this board go to

File ---> preferences ---> additional boards manager url, and add the following: http://download.dfrobot.top/FireBeetle/package_esp32_index.json

then go to

Tools ---> Board ---> Boarads manager and look for FireBeetle-ESP32 install the board

Notice other ESP32 libraries exists but will restuls in a final program to big to be uploaded on the board. 

To receive the data and publish them on ROS it is available a ROS driver:

https://github.com/TheEngineRoom-UniGe/TERGlove_ROS_driver
