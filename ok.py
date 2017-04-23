import serial
import time
import json

import paho.mqtt.client as mqtt
import paho.mqtt.publish as _publish
import hexdump

from datetime import date
from datetime import time
from datetime import datetime

#Read PM6750
TYPE_rECG = 0x02
TYPE_rNIBP = 0x03
TYPE_rSPO2 = 0x04
TYPE_rTEMP = 0x05


#def publish(topic, msg):
def publish(msg):
  output = _publish.single("meditable", payload=msg, qos=0,
      retain=False, hostname="localhost", port=1883,
      keepalive=60, will=None,  tls=None, protocol=mqtt.MQTTv31)

# Generate json sent by mqtt to localhost
#def generate_json(data_type, data_out):
out = ""
json_object = {}


def generate_json(data_out):
  
  # out1 = ""
  # out2 = ""
  # out3 = ""
  # out4 = ""
  global out
  global json_object

  # today = datetime.now()
  # json_object['datetime'] = today

  now = datetime.now()
  json_object['date'] = now.strftime("%A : %d/%B/%y")
  json_object['time'] = now.strftime("%H:%M")


  # ECG Mqtt Sent
  if data_type == TYPE_rECG:
    #json_object = { }
    #json_object['ecg'] = 'ecg'
    #json_object['ecg_sta'] = bin(data_out[0])
    json_object['ecg_heart'] = data_out[1]
    json_object['ecg_resp'] = data_out[2]
    json_object['ecg_stle'] = data_out[3]
    # out1 = json_object


  # NIBP Mqtt Sent
  if data_type == TYPE_rNIBP:
    #json_object = { }
    #json_object['nibp'] = 'nibp'
    #json_object['nibp_sta'] = bin(data_out[1])
    json_object['nibp_sys'] = data_out[2]
    json_object['nibp_dia'] = data_out[3]
    # out2 = json_object


  # SPO2 Mqtt Sent
  status_spo2 = ''
  if data_type == TYPE_rSPO2:
    if data_out[0] == 0:
        status_spo2 = 'Reading'
    elif data_out[0] == 2:
        status_spo2 = 'No finger'
    elif data_out[0] == 3:
        status_spo2 = 'Searching'
    # json_object = { }
    #json_object['spo2'] = 'spo2'
    json_object['spo2_value'] = data_out[1]
    json_object['status'] = status_spo2
    json_object['pulse_value'] = data_out[2]
    # out3 = json_object


  # TEMP Mqtt Sent
  if data_type == TYPE_rTEMP:
    #json_object = { }
    #json_object['temp'] = 'temp'
    json_object['temp_value'] = data_out[1]
    # out4 = json_object

  out = json_object
  return json.dumps(out)

port = serial.Serial("/dev/ttyUSB0",
    baudrate=115200, timeout=3.0,
    # parity=serial.PARITY_NONE,
    # stopbits=serial.STOPBITS_ONE,
    # bytesize=serial.EIGHTBITS
    )



# port = serial.Serial("/dev/tty.BerryMed-DevB",
    # baudrate=115200)
client = mqtt.Client(clean_session=True, userdata=None, protocol=mqtt.MQTTv31)
conn = client.connect("127.0.0.1")



# # Enable ECG
# command = bytearray ([0x55, 0xaa, 0x04, 0x01, 0x01, 0xf9])
# port.write(command)

# # Enable NIBP
# command = bytearray ([0x55, 0xaa, 0x04, 0x02, 0x01, 0xf8])
# port.write(command)

# # Enable SPO2
# command = bytearray ([0x55, 0xaa, 0x04, 0x03, 0x01, 0xf7])
# port.write(command)

# # Enable Temp
# command = bytearray ([0x55, 0xaa, 0x04, 0x04, 0x01, 0xf6])
# port.write(command)
# time.sleep(1)


must_read_hardware = 0;
must_command = 0;
_second = 0;
pevMin = 0;
newMin = 0;

def on_message(mosq, obj, msg):
    global must_read_hardware
    message = msg.payload
    print(msg.topic + " " + message)
    
    if message == "1":
        print "press"
        must_read_hardware = 1     
    else:
        must_read_hardware = 0

  

  
mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.connect("localhost", 1883,60)
mqttc.subscribe("meditable/command", 0)


while True:
    countloop = 0
    mqttc.loop()
    must_command = 1

    now = datetime.now()
    pevMin = now.strftime("%M")
    newMin = int(pevMin) + 2

    print "pre = %s " % now.strftime("%M")
    print "newMin = %d" % newMin

    if newMin >= 60:
        newMin = newMin - 60

    while must_read_hardware == 1:
        if must_command == 1:
            command = bytearray ([0x55, 0xaa, 0x04, 0x01, 0x01, 0xf9])
            port.write(command)
            command = bytearray ([0x55, 0xaa, 0x04, 0x02, 0x01, 0xf8])
            port.write(command)
            command = bytearray ([0x55, 0xaa, 0x04, 0x03, 0x01, 0xf7])
            port.write(command)
            command = bytearray ([0x55, 0xaa, 0x04, 0x04, 0x01, 0xf6])
            port.write(command)
            # time.sleep(1)
            must_command = 0

        
        #print "countloop = %d"% (countloop)
        # now = datetime.now()
        # print "%0.2d" % (now.second)
        # _second = now.second


        now = datetime.now()
        pevMin = now.strftime("%M")
        print "loop = %d"  % newMin
        print  now.strftime("%M")

        if int(pevMin) == newMin:
            print " END "
            must_command = 0
            must_read_hardware = 0

        rcv = port.read()
        if len(rcv) > 0 and ord(rcv) == 0x55:
            if ord(port.read()) == 0xAA:
                data_len = ord(port.read())
                data_type = ord(port.read())
                data_sum = data_type + data_len
                data_out = []
                # 3 bytes for length,
                for l in range(1, data_len-1-1):
                    d = port.read()
                    data_sum += ord(d)
                    data_out.append(ord(d))
                    #print "%s"% (hexdump.dump(d))
                mcu_checksum = ord(port.read())
                our_checksum = (~data_sum & 0xFF)
                if mcu_checksum == our_checksum:
                    if data_type == TYPE_rECG:
                        json_out = generate_json(data_out)
                        publish(json_out)
                        #msgTopic = '/ecg'
                        #publish(msgTopic, json_out)
                    if data_type == TYPE_rNIBP:
                        if data_out[2] >= 1:
                            must_read_hardware = 0
                        json_out = generate_json(data_out)
                        publish(json_out)
                        #msgTopic = '/nibp'
                        #publish(msgTopic, json_out)
                    if data_type == TYPE_rTEMP:
                        json_out = generate_json(data_out)
                        publish(json_out)
                        #msgTopic = '/temp'
                        #publish(msgTopic, json_out)
                    if data_type == TYPE_rSPO2:
                        json_out = generate_json(data_out)
                        publish(json_out)
                        #msgTopic = '/spo2'
                        #publish(msgTopic, json_out)
                #print "sent..."
port.close()
