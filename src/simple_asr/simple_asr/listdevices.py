import sounddevice

devices = sounddevice.query_devices()['index']

print(devices)