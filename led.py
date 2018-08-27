#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import os
import sys
from threading import Timer, Thread, Event
import smbus
import Python_DHT
from INA226 import INA226
#from SUSV import SUSV
from piusv import PiUSV
from mqtt_domoticz import mqttDomoticz
from mqttData import *

U_Batt_low = 3.5
U_ext_low = 10.5

bus = smbus.SMBus(1) # 1 indicates /dev/i2c-1
try:
    bus.read_byte(0x28)
    bus.read_byte(0x48)
    import explorerhat
    eh = 1
    if explorerhat.LED3 == 27:
        explorerhat.LED3 = explorerhat.LED4
        print ("S-USV GPIO27 freed")
except:
    eh = 0
    print ("no exlorerhat detected")
    pass

ina226 = INA226()
if ina226.get_status():
  print("INA226 detected")
else:
  print("INA226 not detected")
	
### S.USV ###
#susv = SUSV()
#print("SUSV Firmware: %s" % susv.get_version())

### PiUPS+ ###
piusv = PiUSV()

class checkTimer():
  def __init__(self,t,hFunction):
    self.t=t
    self.hFunction = hFunction
    self.thread = Timer(self.t,self.handle_function)

  def handle_function(self):
    self.hFunction()
    self.thread = Timer(self.t,self.handle_function)
    self.thread.start()

  def start(self):
    self.thread.start()

  def cancel(self):
    self.thread.cancel()

### mqtt Domoticz ###
def watchdog():
  global wd_timeout
  wd_timeout = wd_timeout - 1
  if wd_timeout == 0:
    print("mqtt connection timeout, restarting script\n")
    # restart led.py
    os.execv(sys.executable, ['python'] + sys.argv)

wd = checkTimer(1, watchdog)
wd_timeout = 10
wd.start() # wait 10 seconds for mqtt connection
mqtt = mqttDomoticz(mqtt_host, mqtt_port, mqtt_user, mqtt_pass, mqtt_cert)
wd.cancel() # stop watchdog
pd = True

# Host als Parameter fuer online Check
if len(sys.argv) == 1:
  host = "8.8.8.8"
else:
  host = str(sys.argv[1])

# Variablen definieren
DHTpin = 18
sensor = Python_DHT.DHT11
StartTime = 0
BtnTimer = 0
offline = 1
# BCM 17 / GPIO 0
#RedLED = 17
# BCM 27 / GPIO 2
#YellowLED = 27
# BCM 22 / GPIO 3
#GreenLED = 22
# BCM 23 / GPIO 4
#Button = 23
# BCM 6 / GPIO 22
RedLED = 6
# BCM 13 / GPIO 23
YellowLED = 13
# BCM 19 / GPIO 24
GreenLED = 19
# BCM 16 / GPIO 27
Button = 16
# exlorer hat
eBlueLED = 0
eYellowLED = 1
eRedLED = 2
eGreenLED = 3

# syslog routine
def log(message):
  os.system("logger -t led " + message)
log("process started, checking host " + host)
  
if eh == 0:
  # GPIO Warnungen ausschalten
  GPIO.setwarnings(False)

  # SoC als Pinreferenz waehlen
  GPIO.setmode(GPIO.BCM)

  # Pin 23 vom SoC als Input deklarieren und Pull-Up Widerstand aktivieren
  GPIO.setup(Button, GPIO.IN, pull_up_down = GPIO.PUD_UP)

  # Pin fuer LEDs vom SoC als Output deklarieren
  GPIO.setup(RedLED,    GPIO.OUT)
  GPIO.setup(YellowLED, GPIO.OUT)
  GPIO.setup(GreenLED,  GPIO.OUT)
  GPIO.output(RedLED, 0)
  GPIO.output(YellowLED, 0)
  GPIO.output(GreenLED, 0)

# ISR
if eh == 0:
  def ButtonEvent(Button):
    # Zugriff auf globale Variablen
    global StartTime, BtnTimer

    if GPIO.input(Button) == 0:
      BtnTimer = 0
      StartTime = time.time()
      GPIO.output(RedLED, 1)
      #log("button pressed")
    elif StartTime > 0:
      BtnTimer = time.time() - StartTime
      StartTime = 0
      GPIO.output(RedLED, 0)
      #log("button released")

if eh == 1:
  ### set sensor timeout of cap1208 reg 0x22 to max = 11200 ms ###
  bus.write_byte_data(0x28, 0x22, bus.read_byte_data(0x28, 0x22) | 0xf0)

  def TouchButtonEvent(channel, event):
    global StartTime, BtnTimer

    if channel > 3:
      return
    if event == 'press':
      print("Button " + str(channel) + " pressed")
      explorerhat.light[channel - 1].on()
      if channel == 1:
        BtnTimer = 0
        StartTime = time.time()
    if event == 'release':
      print("Button " + str(channel) + " released")
      explorerhat.light[channel - 1].off()
      if channel == 1 and StartTime > 0:
        BtnTimer = time.time() - StartTime
        StartTime = 0

  # GPIO27.GEN2 / PIN 13 setzen wegen S-USV
  #print ("S-USV")
  #GPIO.setmode(GPIO.BCM)
  #GPIO.setup(27, GPIO.IN)
  #GPIO.output(27, 1)

# Interrupt Event hinzufuegen. Pin 23 soll auf fallende oder steigende Flanke reagieren und ISR "Interrupt" deklarieren
if eh == 0:
  GPIO.add_event_detect(Button, GPIO.BOTH, callback = ButtonEvent)

if eh == 1:
  explorerhat.touch.pressed(TouchButtonEvent)
  explorerhat.touch.released(TouchButtonEvent)

# check ping thread
def pingcheck():
  global offline, BtnTimer

  offline = os.system("ping -q -w 1 -c 1 " + host + " > /dev/null 2>&1")
  if eh == 0:
    GPIO.remove_event_detect(Button)
    GPIO.add_event_detect(Button, GPIO.BOTH, callback = ButtonEvent)
  if eh == 1:
    #print explorerhat.analog.one.read()
    if GPIO.gpio_function(5) != GPIO.OUT:
      print ("Mist!")
      BtnTimer = 100

def readval():
  global DHTpin, pd

  h1, t1 = Python_DHT.read_retry(sensor, DHTpin)
  if ina226.get_status():
    v1 = ina226.busVoltage()
    c1 = ina226.shuntCurrent()
  else:
    v1 = 0
    c1 = 0
	
  try:
    #piusv.get_parameter()
    #piusv.word2float()
    #s1 = piusv.line()
    pu1 = piusv.U_Batt() / 1000.0
    pu2 = piusv.I_Rasp() / 1000.0
    pu3 = piusv.U_Rasp() / 1000.0
    pu4 = piusv.U_USB() / 1000.0
    pu5 = piusv.U_ext() / 1000.0
  except IOError:
    #s1 = "i2c Error"
    pu1 = 0
    pu2 = 0
    pu3 = 0
    pu4 = 0
    pu5 = 0
    pass
		
  sv1 = 0 # susv.get_voltage()
  sv2 = 0 # susv.get_voltage_bat()
	
  f = open('/tmp/workfile', 'w')
  #f.write("%s\n" % s1)
  f.write("DHT Temperature:  %6.3f C\n" % t1)
  f.write("DHT Humidity:     %6.3f %%\n\n" % h1)
  f.write("INA Voltage:      %6.3f V\n" % v1)
  f.write("INA Current:      %6.3f A\n\n" % c1)
  f.write("SUSV In Voltage:  %6.3f V\n" % sv1)
  f.write("SUSV Bat Voltage: %6.3f V\n\n" % sv2)
  f.write("PiUSV U_Batt:     %6.3f V\n" % pu1)
  f.write("PiUSV I_Rasp:     %6.3f A\n" % pu2)
  f.write("PiUSV U_Rasp:     %6.3f V\n" % pu3)
  f.write("PiUSV U_USB:      %6.3f V\n" % pu4)
  f.write("PiUSV U_ext:      %6.3f V\n" % pu5)
  f.close()
	
  if pd:
    mqtt.publish_temp(t1)
    mqtt.publish_humi(h1)
    mqtt.publish_volt(pu5)
    mqtt.publish_curr(pu2)
    mqtt.publish_batt(pu1)
    pd = False

  shutdown = 0
  for i in range (5):
    if (pu5 <= U_ext_low) and (pu5 >= 6.0):
      shutdown = shutdown + 1
      pu5 = piusv.U_ext() / 1000.0
  if shutdown == 5:
    # external battery low -> shutdown
    mqtt.publish_volt(pu5)
    time.sleep(5)
    os.system("sudo /usr/local/bin/stopusv")

  shutdown = 0
  for i in range(5):
    if (pu1 <= U_Batt_low) and (pu4 <= U_Batt_low) and (pu5 <= U_Batt_low):
      shutdown = shutdown + 1
      pu1 = piusv.U_Batt() / 1000.0
      pu4 = piusv.U_USB() / 1000.0
      pu5 = piusv.U_ext() / 1000.0
  if shutdown == 5:
    # no external power and usv battery low -> shutdown
    mqtt.publish_batt(pu1)
    time.sleep(5)
    os.system("sudo /usr/local/bin/stopusv")
		
def publish_data():
  global pd
	
  pd = True

# Endlosschleife
try:
  t = checkTimer(10, pingcheck)
  t.start()
  t1 = checkTimer(16, readval)
  t1.start()
  t2 = checkTimer(301, publish_data)
  t2.start()
  while BtnTimer <= 7:
    if offline:
      # falls offline gruene LED schnell blinken
      if eh == 0:
        GPIO.output(GreenLED, 1)
      if eh == 1:
        explorerhat.light[eGreenLED].on()
      time.sleep(0.001)
      if eh == 0:
        GPIO.output(GreenLED,0)
      if eh == 1:
        explorerhat.light[eGreenLED].off()
      time.sleep(0.4)
    else:
      # falls online gruene LED langsam blinken
      if eh == 0:
        GPIO.output(GreenLED,1)
      if eh == 1:
        explorerhat.light[eGreenLED].on()
      time.sleep(0.01)
      if eh == 0:
        GPIO.output(GreenLED,0)
      if eh == 1:
        explorerhat.light[eGreenLED].off()
      time.sleep(2)

    # Button laenger als 3 Sekunden gedrueckt, gelbe LED an
    if (StartTime > 0) and (time.time() - StartTime) >= 3:
      if eh == 0:
        GPIO.output(YellowLED, 1)
      if eh == 1:
        explorerhat.light[eYellowLED].on()

    # Button laenger als 7 Sekunden gedrueckt, gelbe LED aus
    if (StartTime > 0) and (time.time() - StartTime) >= 7:
      if eh == 0:
        GPIO.output(YellowLED, 0)
      if eh == 1:
        explorerhat.light[eYellowLED].off()

    # Button kuerzer als 1 Sekunde gedrueckt, netcheck
    if (BtnTimer > 0) and (BtnTimer < 1):
      log("button pressed " + str(BtnTimer) + " seconds - netcheck")
      BtnTimer = 0
      os.system("sudo /usr/local/bin/netcheck")

    # Button mindestens 3, jedoch unter 7 Sekunden gedrueckt, gelbe LED aus, Netzwerkneustart
    if (BtnTimer >= 3) and (BtnTimer < 7):
      log("button pressed " + str(BtnTimer) + " seconds - restarting network")
      BtnTimer = 0
      os.system("sudo service networking restart")
      os.system("sudo service isc-dhcp-server restart")
      if eh == 0:
        GPIO.output(YellowLED, 0)
      if eh == 1:
        explorerhat.light[eYellowLED].off()

except KeyboardInterrupt:
  print ("\nCTRL+C")

GPIO.cleanup()
#t.cancel()
print ("Beendet")

if BtnTimer == 100:
  t.cancel()
  t1.cancel()
  t2.cancel()
  os.execv(sys.executable, ['python'] + sys.argv)

# Button laenger als 7 Sekunden gedrueckt, poweroff
if BtnTimer >= 7:
  print ("Button pressed " + str(BtnTimer) + " seconds")
  log("Button pressed " + str(BtnTimer) + " seconds - poweroff initiated")
  #os.system("sudo poweroff")
  os.system("sudo /usr/local/bin/stopusv")
t.cancel()
t1.cancel()
t2.cancel()
