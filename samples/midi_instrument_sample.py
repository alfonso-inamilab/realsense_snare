# HOW TO SWITCH MIDI INSTRUMENT using mido
import mido
import time

# OPEN MIDID port with mido 
print ( mido.get_output_names() )  # Prints MIDI port names
out_port = mido.open_output('loopPort 1')  # Edit this with your own Loop MIDI port name 

# Choose instrument
pc = mido.Message('program_change', program=36, channel=1, time=0)
out_port.send(pc)

msg = mido.Message('note_on', channel=1, note=60, velocity=127, time=0)  # Create MIDI ON message with mido
out_port.send(msg) # Send the note though to the MIDI device (in my case LoopMIDI -> Virtual Midi Piano)
time.sleep(0.5)   # Wait for 500ms
msg = mido.Message('note_off', channel=2, note=60, velocity=127, time=0)  # Create MIDI OFF message with mido
out_port.send(msg)  # Send the note off though to the MIDI device 
time.sleep(1)   # Wait for first instrument to stop

# Change instrument again 
pc = mido.Message('program_change', program=40, channel=1, time=0)
out_port.send(pc)

msg = mido.Message('note_on', channel=1, note=60, velocity=127, time=0)  # Create MIDI ON message with mido
out_port.send(msg) # Send the note though to the MIDI device (in my case LoopMIDI -> Virtual Midi Piano)
time.sleep(0.5)   # Wait for 500ms
msg = mido.Message('note_off', channel=2, note=60, velocity=127, time=0)  # Create MIDI OFF message with mido
out_port.send(msg)  # Send the note off though to the MIDI device 
time.sleep(1)   # Wait for second instrument to stop



