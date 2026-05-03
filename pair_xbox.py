#
# Name: pair_xbox.py
#
# Description:
# A little program to pair the XboxController with the hub.
# After properliy paired, the hub light will blink on green and then exit.
#
# Notice that on Technic Hubs, if started form the computer, the hub will disconnect from 
# the computer before connecting to the Xbox (disconnects on the call to XboxController()).
#
# Version: 1.0
#
# Author VascoLP: vascolp.lego@gmail.com
#
# Date: April 2026
#


from pybricks.hubs import ThisHub
from pybricks.iodevices import XboxController
from pybricks.parameters import Color
from pybricks.tools import wait

hub = ThisHub()
hub.light.on(Color.WHITE)
print('Connect XboxController...')
try:
    xbox=XboxController()
    print('ok')
    hub.light.on(Color.GREEN)
    wait(2000)
except OSError as ex:
    print('kabum')
    hub.light.on(Color.RED)
    wait(2000)



