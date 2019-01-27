I'm using a plugin for atom called platform.io. This plugin allows me to upload and debug arduino programs from within the atom editor. Note: you will need to open 3 projects in atom: the robo-boat project, and the arduino project. Platformio will run whatever project you selected last, so you need to open a file from one of the services and then upload/run.

Pins:
  Location service arduino:
    GPS connects TX  to pin 3.
    CP  connects SDA to pin A4.
    CP  connects SDL to pin A5.
  Motor service arduino:
    RM ESC to pin 2.
    LM ESC to pin 3.
