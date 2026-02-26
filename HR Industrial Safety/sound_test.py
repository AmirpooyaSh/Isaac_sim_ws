import subprocess
import tempfile
import os

VR_SINK = "alsa_output.usb-HTC_VIVE_Pro_Mutimedia_Audio-00.analog-stereo"

# Create temporary wav file
with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmp:
    tmp_filename = tmp.name

# Generate a 1-second 440Hz tone using sox
subprocess.run([
    "sox", "-n", "-r", "44100", "-c", "2",
    tmp_filename, "synth", "1", "sine", "440"
])

# Play it on the VR headset
subprocess.run([
    "paplay",
    "--device", VR_SINK,
    tmp_filename
])

# Cleanup
os.remove(tmp_filename)