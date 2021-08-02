# FOR MIDI MESSAGES CREATION AND CONTROL
import mido
import time

# OPEN MIDID port with mido 
print ( mido.get_output_names() )  # Prints MIDI port names
out_port = mido.open_output('loopPort 1')  # Edit this with your own Loop MIDI port name 

# Create MIDI ON message with mido
msg = mido.Message('note_on', channel=1, note=60, velocity=127, time=0)
# Send the note though to the MIDI device (in my case LoopMIDI -> Virtual Midi Piano)
out_port.send(msg)

# Wait for 500ms 
time.sleep(0.5)

# Create MIDI OFF message with mido
msg = mido.Message('note_off', channel=1, note=60, velocity=127, time=0)
# Send the note off though to the MIDI device 
out_port.send(msg)