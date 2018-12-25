Location service and motor service are two projects, for the two arduinos onboard.

The location service will constantly send information to the jetson, so it can just pull from that serial port whenever it needs GPS or compass data.

The motor service will never send any information to the jetson, but will just take commands from the jetson or RC receiver, and control the motors.

Pins:
  Location service arduino:
    GPS connects TX  to pin 3.
    CP  connects SDA to pin A4.
    CP  connects SDL to pin A5.
  Motor service arduino:
    RM ESC to pin 2.
    LM ESC to pin 3.
