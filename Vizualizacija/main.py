from TemperaturePoint import TemperaturePoint
from Plot3D import Plot3D
import paho.mqtt.client as mqtt
import re

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("TemperaturePoint")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    data = re.search("\'x=(.*) y=(.*) z=(.*) temp=(.*)\'", str(msg.payload)).groups()
    plot3D.update(TemperaturePoint(float(data[0]), float(data[1]), float(data[2]), float(data[3])))

plot3D = Plot3D()

mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect("localhost", 1883, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
mqttc.loop_forever()