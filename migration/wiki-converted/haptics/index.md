### Using Audacity### 
Audacity is used for the producing of multi-channel track/channel audio files and producing music. For our haptics purposes we're using Audacity to produce our own 8 channel audio files to play on the Woojer Haptics Vest.
  - The first step to using Audacity is to configure your settings to allow you to export files in multiple channels and to stop Audacity from automatically converting files into stereo or mono. The way to do this is to go to your audacity preferences>import/export and select "Use custom mix" instead of "Always mix all tracks down to stereo or mono channels" which is the default.                                                    {{:screen_shot_2019-10-17_at_6.27.03_pm.png?400|}}
  - The second step will be to find an audio file to use. Currently we have been using surround sound audio files from the BBC Sound Effect Library (http://bbcsfx.acropolis.org.uk/) and importing them (drag and drop) into Audacity. Once you have an audio clip inside of Audacity it will show up as its own track. Tracks are the paths that hold each audio file we want to add to our final larger audio file. If, when you import the sound file, the track shows up as a stereo track, change it to a mono track by clicking the drop down error and choosing split to mono track. For our purposes we want all our tracks to be mono tracks. We will need to have 8 tracks total to assign to the 8 different channels so you will need to add in tracks by clicking tracks>add new>mono track.
  - The third step is to export the file. To do this we will go to file>export audio and save the file (make sure that you name your file something relevant and save it to a good location). Once you have saved it you should be presented with a menu that looks like this:{{::screen_shot_2019-10-17_at_6.49.56_pm.png?400|}}                                      
This menu will allow you to assign your tracks to different channels, also on this menu we will move the slider at the bottom to 8 channels. Each channel corresponds to a transducer on the vest as seen in the diagram below from the Woojer Haptics Vest manual.                                   
{{::image_3_.png?400|}} 
  - For example: If we want to produce a vibration on T1, we will assign each audio track to channel 1 on the menu. After doing this, press "ok" to save the file. The result from assigning these tracks will look something like this:        
{{::screen_shot_2019-10-17_at_6.58.50_pm.png?400|}}                                                                     












### Playing Sounds With Python### 
import sounddevice as sd 

import soundfile as sf

data, fs = sf.read(r"filename",dtype='float32') 

sd.query_devices() 

sd.play(data,fs,device=int) 

sd.stop()