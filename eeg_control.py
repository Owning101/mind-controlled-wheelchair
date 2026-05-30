# eeg_testdelayed.py
# Robust Muse 2 EEG reader with 0.5s delay, auto-reconnect, and manual stop

from pylsl import StreamInlet, resolve_streams
import numpy as np
import time
import threading
import subprocess
import time

print("Starting Muse stream...")

# Start muselsl stream in background
subprocess.Popen(["muselsl", "stream"])

# Give it time to connect
time.sleep(8)

DELAY_SECONDS = 0.5  # Delay between readings
running = True        # Control flag for the loop

def stop_listener():
    """Thread to listen for 'D' key to stop the script"""
    global running
    while running:
        user_input = input()
        if user_input.strip().upper() == 'D':
            print("Stopping script...")
            running = False
            break

# Start the listener thread
threading.Thread(target=stop_listener, daemon=True).start()

while running:
    try:
        print("Looking for EEG stream...")
        streams = resolve_streams()  # get all streams

        eeg_stream = None
        for s in streams:
            if s.type() == 'EEG':
                eeg_stream = s
                break

        if eeg_stream is None:
            print("No EEG stream found. Retrying in 2 seconds...")
            time.sleep(2)
            continue

        inlet = StreamInlet(eeg_stream)
        print("Connected! Receiving data...\n")

        while running:
            sample, timestamp = inlet.pull_sample(timeout=1.0)  # wait max 1 sec
            if sample is None:
                # No data available
                print("TP9:None | AF7:None | AF8:None | TP10:None")
            else:
                eeg = np.array(sample[:4])  # first 4 channels
                print(f"TP9:{eeg[0]:.2f} | AF7:{eeg[1]:.2f} | AF8:{eeg[2]:.2f} | TP10:{eeg[3]:.2f}")

            time.sleep(DELAY_SECONDS)

    except KeyboardInterrupt:
        print("")
        running = False
    except Exception as e:
        print(f"Error: {e}")
        print("Retrying connection in 2 seconds...")
        time.sleep(2)

print("Script terminated.")