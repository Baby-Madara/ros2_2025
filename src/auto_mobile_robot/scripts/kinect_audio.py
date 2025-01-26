#!/usr/bin/env python3

import freenect
import time
import wave
import pyaudio

# Function to set the tilt of the Kinect motor
def set_tilt_angle(device, angle):
    if -30 <= angle <= 30:  # Kinect's tilt range
        freenect.set_tilt_degs(device, angle)
        print(f"Set Kinect tilt angle to {angle} deg.")
    else:
        print("Angle out of range. Valid range is -30 to 30 degrees.")

# Function to get the current tilt angle
def get_tilt_angle(device):
    state = freenect.get_tilt_state(device)
    tilt_angle = freenect.get_tilt_degs(state)
    print(f"Current tilt angle: {tilt_angle} deg.")
    return tilt_angle

# Function to record audio from Kinect microphones
def record_audio(filename, record_seconds=5, chunk_size=1024):
    p = pyaudio.PyAudio()
    
    # Kinect microphones support 4 channels, 16-bit samples, 16 kHz sampling rate
    stream = p.open(format=pyaudio.paInt16,
                    channels=4,  # Use 4 channels for Kinect microphones
                    rate=16000,
                    input=True,
                    frames_per_buffer=chunk_size)

    print("Recording audio from 4 microphones...")
    frames = []

    for _ in range(0, int(16000 / chunk_size * record_seconds)):
        data = stream.read(chunk_size, exception_on_overflow=False)
        frames.append(data)

    print("Recording complete.")

    stream.stop_stream()
    stream.close()
    p.terminate()

    # Save audio to a WAV file
    wf = wave.open(filename, 'wb')
    wf.setnchannels(4)  # 4 channels
    wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
    wf.setframerate(16000)
    wf.writeframes(b''.join(frames))
    wf.close()
    print(f"Audio saved to {filename}.")

# Main program
if __name__ == "__main__":
    try:
        # Open Kinect device
        device = freenect.open_device(freenect.init(), 0)

        # Control the Kinect servo
        set_tilt_angle(device, 0)  # Set tilt to 0 degrees
        time.sleep(2)  # Allow time for the servo to move
        current_angle = get_tilt_angle(device)

        # Record audio
        record_audio("kinect_audio_4ch.wav", record_seconds=10)

        # Close Kinect device
        freenect.close_device(device)

    except KeyboardInterrupt:
        print("\nExiting program.")
    except Exception as e:
        print(f"An error occurred: {e}")
