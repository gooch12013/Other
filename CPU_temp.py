import serial
import wmi
import time
import serial

ser = serial.Serial ('COM1')

ser.baudrate = 9600

ser.write('vis t0,1\xFF\xFF\xFF')
ser.write('t0.txt="CPU Temp"\xFF\xFF\xFF')

w = wmi.WMI(namespace="root/OpenHardwareMonitor")

while 1==1:
  temperature_infos = w.Sensor()
  for sensor in temperature_infos:
    if sensor.SensorType==u'Temperature' and sensor.Name=='CPU Package':
      val = 'z'
      if sensor.Value < 60:
        val = 'x'
      elif sensor.Value < 80:
        val = 'y'

      print(sensor.Value)
      ser.write('t0.txt="%s"\xFF\xFF\xFF' % sensor.Value)
      ser.write('z0.val="%s"\xFF\xFF\xFF' % sensor.Value)
      #print(val)

  time.sleep( .5 )
