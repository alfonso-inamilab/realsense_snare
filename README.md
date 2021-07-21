# RealSense Snare
Snare and cymbals hit prediction sensor using a **single** RealSense depth camera

### Prototype
- `prototype/rs_snare.py` prototype for the hit detection uses aruco and plays detections with MIDI

### Samples
- `samples/midi_output_sample.py` sample to send midi output commands with mido (requires conection with MIDI synth)
- `samples/midi_instrument_sample.py` how to change midi instrument (requires conection with MIDI synth)
- `samples/sample_aruco.py`  snare hit detection using aruco markers  
- `samples/sample.py`  easiest hit detection with no aruco markers only surface depths (do not work with reflective surfaces)

## Libraries 
To run the samples and the prototype you need to install the following libraries:

RealSense python libraries
````
pip install pyrealsense2
````

OpenCV libraries (with aruco support)
````
pip install opencv-python
pip install opencv-contrib-python
````

Python mido libraries
````
pip install mido
pip install python-rtmidi
````
## Connecting the MIDI synthesizer 

## TODO tasks
There is not velocity measurement yet. Altough Iiyama-san Arduino/accelerometer stick could easily solve this problem. 