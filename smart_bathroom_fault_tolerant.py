# smart_bathroom_fault_tolerant_impl.py
# Fault-tolerant Smart Bathroom (Pico W) -- IR stuck detection + WDT & robust error handling


from mqtt_as import MQTTClient, config
from machine import Pin, WDT, reset, reset_cause
import uasyncio as asyncio
import utime, micropython
from dht import DHT11

PIN_IR_PRIMARY = 0    # primary IR OUT
PIN_IR_SECOND = 14    # secondary IR OUT 
PIN_LED = 16
PIN_FAN = 22
PIN_DHT = 4

# thresholds & timing (ms)
DEBOUNCE_MS = 500            # debounce per trigger
STUCK_MS = 60_000            # if sensor stuck same state for this long -> flagged (60s)
WDT_TIMEOUT_MS = 8000        # watchdog timeout (8s)
MQTT_RETRY_LIMIT = 4         # attempts before raising an error/fallback
HEARTBEAT_INTERVAL = 15      # seconds
HUM_THRESHOLD = 70           # % humidity to force fan on

# MQTT topics
TOPIC_OCCUPANCY = "bathroom/occupancy"
TOPIC_TEMP = "bathroom/temperature"
TOPIC_HUM = "bathroom/humidity"
TOPIC_FAN_STATUS = "bathroom/fan/status"
TOPIC_HEARTBEAT = "bathroom/heartbeat"
TOPIC_FAULTS = "bathroom/faults"
TOPIC_BOOT = "bathroom/boot"

# WiFi / MQTT broker (edit for your network)
WIFI_SSID = "iotlab"
WIFI_PW = "modermodemet"
config.update({
    "ssid": WIFI_SSID,
    "wifi_pw": WIFI_PW,
    "server": "test.mosquitto.org",
    "port": 1883,
    "user": "",
    "password": "",
    "queue_len": 1,
})
MQTTClient.DEBUG = False  

led = Pin(PIN_LED, Pin.OUT)
fan = Pin(PIN_FAN, Pin.OUT)
dht = DHT11(Pin(PIN_DHT))

ir_primary_pin = Pin(PIN_IR_PRIMARY, Pin.IN, Pin.PULL_UP)
ir_second_pin = None
if PIN_IR_SECOND is not None:
    ir_second_pin = Pin(PIN_IR_SECOND, Pin.IN, Pin.PULL_UP)

occupied = False
humidity = None
temperature = None
last_trigger_time = 0         # for debounce primary/secondary (ms)
last_change_primary = utime.ticks_ms()
last_change_secondary = utime.ticks_ms()
primary_stuck = False
secondary_stuck = False
last_primary_value = ir_primary_pin.value()
last_secondary_value = ir_second_pin.value() if ir_second_pin else None

# events handling (keep interrupts tiny)
events = []  
event_pending = False

def now_ms():
    return utime.ticks_ms()

def scheduled_add_event(arg):
    # arg = 1 for primary, 2 for secondary
    global event_pending
    t = now_ms()
    sensor = 'P' if arg == 1 else 'S'
    events.append((sensor, t))
    event_pending = True

def make_irq_handler(id_int):
    def handler(pin):
        t = now_ms()
        # simple debounce per-sensor: check last change recorded
        global last_trigger_time
        if utime.ticks_diff(t, last_trigger_time) > DEBOUNCE_MS:
            last_trigger_time = t
            micropython.schedule(scheduled_add_event, id_int)
    return handler

# attach IRQs
ir_primary_pin.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=make_irq_handler(1))
if ir_second_pin:
    ir_second_pin.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=make_irq_handler(2))

# subs_cb is synchronous callback used by mqtt_as for incoming messages
def subs_cb(topic, msg, retained):
    # Keep tiny: decode and set overrides if needed (not used here)
    t = topic.decode()
    p = msg.decode()
    # example: we could implement remote fan override here
    # but keep minimal for safety in fault demo
    print("MQTT recv", t, p)

config["subs_cb"] = subs_cb
client = MQTTClient(config)

# Create WDT; feed periodically in main loop and long-running tasks
wdt = WDT(timeout=WDT_TIMEOUT_MS)


def publish_boot_reason():
    try:
        rc = reset_cause()
        # microcontroller reset cause values vary; just publish integer and string
        reason_str = str(rc)
        # publish non-blocking attempt (must be awaited in async context normally),
        # but we prepare a small message to publish shortly after connect in main.
        return {"reset_cause": rc, "reset_str": reason_str}
    except Exception:
        return {"reset_cause": "unknown"}

boot_info = publish_boot_reason()


def process_events_and_toggle():
    """
    Called from the main loop (not IRQ). Will process events FIFO.
    For single-sensor toggle approach: primary toggles occupancy.
    If primary is flagged stuck and secondary exists, use secondary toggles.
    """
    global events, occupied, event_pending, last_change_primary, last_change_secondary
    if not events:
        return
    i = 0
    while i < len(events):
        sensor, t = events[i]
        if sensor == 'P':
            # read current value (active low)
            v = ir_primary_pin.value()
            # toggle occupancy on FALLING edge (1->0) only to avoid double toggles on bounces
            if v == 0:
                occupied = not occupied
                last_change_primary = now_ms()
                print("Primary IR event: toggled occupied ->", occupied)
        elif sensor == 'S' and ir_second_pin:
            v = ir_second_pin.value()
            if v == 0:
                # if primary stuck, use secondary to toggle; otherwise just ignore or log
                if primary_stuck:
                    occupied = not occupied
                    last_change_secondary = now_ms()
                    print("Secondary IR event used (primary stuck): occupied ->", occupied)
                else:
                    # secondary event received but primary healthy: we can log or use for verification
                    last_change_secondary = now_ms()
                    print("Secondary IR event (verified).")
        i += 1

    # clear processed events
    events = []
    event_pending = False


def update_stuck_status():
    """
    Update primary_stuck / secondary_stuck flags by checking whether a sensor value
    stayed constant for longer than STUCK_MS and stays in a triggered state.
    """
    global primary_stuck, secondary_stuck, last_primary_value, last_secondary_value
    tnow = now_ms()
    # primary
    pv = ir_primary_pin.value()
    if pv != last_primary_value:
        last_primary_value = pv
        # reset last_change_primary time to now
        # using this to say sensor changed state recently
        # last_change_primary set in event processing as well
        # (no extra action needed)
    else:
        # check how long it's been since last change
        if utime.ticks_diff(tnow, last_change_primary) > STUCK_MS:
            # if value indicates triggered (active low) or constant suspiciously, flag
            if pv == 0:
                if not primary_stuck:
                    primary_stuck = True
                    print("Primary IR flagged STUCK (value 0).")
        else:
            primary_stuck = False

    # secondary
    if ir_second_pin:
        sv = ir_second_pin.value()
        if sv != last_secondary_value:
            last_secondary_value = sv
        else:
            if utime.ticks_diff(tnow, last_change_secondary) > STUCK_MS:
                if sv == 0:
                    if not secondary_stuck:
                        secondary_stuck = True
                        print("Secondary IR flagged STUCK (value 0).")
                else:
                    pass
            else:
                secondary_stuck = False

def set_actuators_and_local_state():
    global humidity
    if humidity is None:
        # if humidity unknown, rely only on occupancy
        fan_state = bool(occupied)
    else:
        fan_state = bool(occupied) or (humidity >= HUM_THRESHOLD)
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
            # print("DHT:", temperature, humidity)
        except Exception as e:
            print("DHT read failed:", e)
        await asyncio.sleep(10)

async def heartbeat_task(mqtt_client):
    while True:
        payload = {
            "occupied": occupied,
            "humidity": humidity,
            "temperature": temperature,
            "primary_stuck": primary_stuck,
            "secondary_stuck": secondary_stuck,
            "boot_info": boot_info
        }
        try:
            await mqtt_client.publish(TOPIC_HEARTBEAT, str(payload), qos=0)
        except Exception as e:
            print("Heartbeat publish error:", e)
        await asyncio.sleep(HEARTBEAT_INTERVAL)

async def publish_task(mqtt_client):
    last_occ = None
    while True:
        # publish sensor values and fan state
        try:
            await mqtt_client.publish(TOPIC_HUM, str(humidity), qos=0)
            await mqtt_client.publish(TOPIC_TEMP, str(temperature), qos=0)
            await mqtt_client.publish(TOPIC_FAN_STATUS, str(int(fan.value())), qos=0)
            # publish occupancy only on change
            if occupied != last_occ:
                await mqtt_client.publish(TOPIC_OCCUPANCY, "1" if occupied else "0", qos=1)
                last_occ = occupied
            # if sensor stuck flags set, publish fault message
            if primary_stuck:
                await mqtt_client.publish(TOPIC_FAULTS, "primary_ir_stuck", qos=1)
            if secondary_stuck:
                await mqtt_client.publish(TOPIC_FAULTS, "secondary_ir_stuck", qos=1)
        except Exception as e:
            # network error: print and continue; WDT will reset if stuck later
            print("Publish error:", e)
        await asyncio.sleep(2)


async def main():
    # connect mqtt_as (which manages Wi-Fi). We rely on mqtt_as to handle Wi-Fi reconnects.
    try:
        await client.connect()
        await client.up.wait()
        print("Connected to MQTT broker and Wi-Fi")
        # publish boot info
        try:
            await client.publish(TOPIC_BOOT, str(boot_info), qos=0)
        except:
            pass
    except Exception as e:
        print("MQTT connect failed:", e)
        # if connection fails repeatedly we still start the rest, publishes will fail until connected
    # start background tasks
    asyncio.create_task(dht_task())
    asyncio.create_task(heartbeat_task(client))
    asyncio.create_task(publish_task(client))

    # main reactive loop
    try:
        while True:
            # feed the watchdog early in loop
            try:
                wdt.feed()
            except Exception:
                pass

            # process any scheduled IR events (safe, non-IRQ)
            if event_pending:
                process_events_and_toggle()

            # check stuck sensors and update flags
            update_stuck_status()

            # set actuators
            fan_state = set_actuators_and_local_state()

            # If primary stuck and secondary exists, ensure we use secondary for occupancy decisions
            # (Handled in event processing where secondary is used if primary_stuck==True)

            # Keep loop responsive
            await asyncio.sleep_ms(100)
    except Exception as e:
        print("Main loop exception:", e)
        # If we get here, allow WDT to handle reset or do a controlled reset after some delay
        await asyncio.sleep(1)
        try:
            reset()
        except:
            print("ERROR")

try:
    asyncio.run(main())
finally:
    try:
        client.close()
    except:
        pass
    try:
        asyncio.new_event_loop()
    except:
        print("ERROR")
