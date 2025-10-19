# smart_bathroom_impl.py
# Smart Bathroom Monitor (Pico W) before fault tolerance strategies

from mqtt_as import MQTTClient, config
from machine import Pin
import network, uasyncio as asyncio, utime, micropython
from dht import DHT11

PIN_IR = 0         # IR sensor
PIN_LED = 16       # LED
PIN_FAN = 22       # Fan
PIN_DHT = 4        # DHT11 sensor

led = Pin(PIN_LED, Pin.OUT)
fan = Pin(PIN_FAN, Pin.OUT)
ir = Pin(PIN_IR, Pin.IN, Pin.PULL_UP)  # active LOW output
dht = DHT11(Pin(PIN_DHT))

HUM_THRESHOLD = 70     # humidity % to auto-run fan
HEARTBEAT_INTERVAL = 15  # seconds between MQTT heartbeat
DEBOUNCE_MS = 1000       # 1 second debounce per trigger

wifi_ssid = "iotlab"
wifi_password = "modermodemet"

config.update({
    "server": "test.mosquitto.org",
    "port": 1883,
    "user": "",
    "password": "",
    "ssid": wifi_ssid,
    "wifi_pw": wifi_password,
    "queue_len": 1,
})
MQTTClient.DEBUG = True
client = MQTTClient(config)

TOPIC_OCCUPANCY = "bathroom/occupancy"
TOPIC_TEMP = "bathroom/temperature"
TOPIC_HUM = "bathroom/humidity"
TOPIC_FAN_STATUS = "bathroom/fan/status"
TOPIC_HEARTBEAT = "bathroom/heartbeat"


occupied = False
humidity = 0
temperature = 0
last_trigger_time = 0
fan_override = None  # None = auto, True/False = forced


async def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(wifi_ssid, wifi_password)
    print("Connecting to WiFi...")
    for _ in range(15):
        if wlan.isconnected():
            print("WiFi connected:", wlan.ifconfig()[0])
            break
        await asyncio.sleep(1)
    if not wlan.isconnected():
        raise RuntimeError("WiFi connection failed")
    await client.connect()
    print("MQTT connected")

def update_actuators():
    global occupied, humidity, fan_override
    if fan_override is not None:
        fan_state = fan_override
    else:
        fan_state = occupied or humidity >= HUM_THRESHOLD

    led.value(1 if occupied else 0)
    fan.value(1 if fan_state else 0)
    return fan_state

async def dht_task():
    global humidity, temperature
    while True:
        try:
            dht.measure()
            temperature = dht.temperature()
            humidity = dht.humidity()
            print(f"DHT11: {temperature}°C, {humidity}%")
        except Exception as e:
            print("DHT read error:", e)
        await asyncio.sleep(10)

async def heartbeat_task():
    while True:
        payload = {
            "occupied": occupied,
            "temp": temperature,
            "hum": humidity,
            "fan": fan.value()
        }
        try:
            await client.publish(TOPIC_HEARTBEAT, str(payload), qos=0)
        except Exception as e:
            print("Heartbeat publish error:", e)
        await asyncio.sleep(HEARTBEAT_INTERVAL)

def ir_trigger(pin):
    global last_trigger_time, occupied
    current = utime.ticks_ms()
    if utime.ticks_diff(current, last_trigger_time) > DEBOUNCE_MS:
        occupied = not occupied  # Toggle state
        print("IR triggered → Occupied =", occupied)
        last_trigger_time = current
        micropython.schedule(lambda x: None, 0)  # yield back to main loop

# Attach interrupt
ir.irq(trigger=Pin.IRQ_FALLING, handler=ir_trigger)

async def main(client):
    await wifi_connect()
    asyncio.create_task(dht_task())
    asyncio.create_task(heartbeat_task())

    last_occupancy = occupied

    while True:
        fan_state = update_actuators()

        # If occupancy changed, publish immediately
        if occupied != last_occupancy:
            await client.publish(TOPIC_OCCUPANCY, "1" if occupied else "0", qos=1)
            print("Published occupancy =", occupied)
            last_occupancy = occupied

        try:
            await client.publish(TOPIC_TEMP, str(temperature), qos=0)
            await client.publish(TOPIC_HUM, str(humidity), qos=0)
            await client.publish(TOPIC_FAN_STATUS, str(int(fan_state)), qos=0)
        except:
            print("ERROR")

        await asyncio.sleep(2)

try:
    asyncio.run(main(client))
finally:
    client.close()
    try:
        asyncio.new_event_loop()
    except:
        print("ERROR")

