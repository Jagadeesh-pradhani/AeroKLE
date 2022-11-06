# AeroKLE

## Pixhawk and Raspberrypi Integration

1. **Connect to Raspberry pi**
   1. Install RPI software using “Imager” to SD card.
   2. Connect Pi SSH using Wi-Fi.
   3. Run the Following commands,
      ```
      sudo apt-get update
      sudo apt-get upgrade
      sudo apt-get install python-pip
      sudo apt-get install python-dev
      sudo pip install future
      sudo apt-get install screen
      sudo apt-get install python-wxgtk4.*
      sudo apt-get install libxml 
      sudo apt-get install libxml2-dev
      sudo apt-get install libxslt1-dev
      pip install lxml
      sudo pip3 install pyserial
      sudo pip3 install dronekit
      sudo pip3 install geopy
      sudo pip3 install MAVProxy
      ```
    4. Set up serial connection, type following in ssh
       ```
       sudo raspi-config
       ```
        1. goto interface options,
        2. go to serial,
        3. When prompted, select  no  to “Would you like a login shell to be accessible over serial?”
        4. When prompted, select  yes  to “Would you like the serial port hardware to be enabled?”.
        5. Reboot the Raspberry Pi when you are done.
        6. The Raspberry Pi’s serial port will now be usable on ``` /dev/serial0 ```.
       
   5. Set following parameters in mission planner,  
      ```SERIAL2_PROTOCOL = 2```  
      ```SERIAL2_BAUD = 921```  
      if required do following also,  
      ```LOG_BACKEND_TYPE = 3``` 
      
   6. Now connect Pixhawk and Raspberry pi, as shown in,
      ![pi-hawk](https://discuss.ardupilot.org/uploads/default/original/2X/f/f837b6b1116ec02c3490e34035c2f09da5a62936.jpg)
   7. Power the RPI using BEC module.
      1. check port,
         ```
         ls /dev/ttyAMA0
         ```
      2. add below two lines at bottom of file ```sudo nano /boot/config.txt``` ,if not there
         ```
            enable_uart=1
            dtoverlay=disable-bt
         ```
   8. Now type the following to get the telemetry data of pixhawk,
      ```
      mavproxy.py --master=/dev/serial0 --baudrate 921600
                  (or)
      mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600
      ```
   9. Type the following if you want telemetry data to be displayed in mission planner,
      ```
      mavproxy.py --master=/dev/serial0 --baudrate 921600 --out udp:127.0.0.1:14552
      
      /*Here,
       '127.0.0.1' Your PC's IP Adress, Obtained by typing 'ipconfig' in command prompt
       '14552' is the port to which you need to connect to mission planner using UDP
      */
     ```
  2. **To Run Python code type,**
       ```
       python3 demo.py --connect /dev/ttyAMA0
       ```
