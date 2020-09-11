# -*- coding: utf-8 -*-
"""
Created on Wed Apr  8 21:18:24 2020

@author: boein
"""

import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import threading
import speech_recognition as sr
# import thread

minimal_create_interface="""
service experimental.minimal_create

object create_obj
    function void run()
    function void StartStreaming()
    property int32 read
    
end object
"""



def recognize_speech_from_mic(recognizer, microphone):
    """Transcribe speech from recorded from `microphone`.

    Returns a dictionary with three keys:
    "success": a boolean indicating whether or not the API request was
               successful
    "error":   `None` if no error occured, otherwise a string containing
               an error message if the API could not be reached or
               speech was unrecognizable
    "transcription": `None` if speech could not be transcribed,
               otherwise a string containing the transcribed text
    """
    # check that recognizer and microphone arguments are appropriate type
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")

    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    # adjust the recognizer sensitivity to ambient noise and record audio
    # from the microphone
    with microphone as source:
        # recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    # set up the response object
    response = {
        "success": True,
        "error": None,
        "transcription": None
    }

    # try recognizing the speech in the recording
    # if a RequestError or UnknownValueError exception is caught,
    #     update the response object accordingly
    try:
        response["transcription"] = recognizer.recognize_google(audio)
    except sr.RequestError:
        # API was unreachable or unresponsive
        response["success"] = False
        response["error"] = "API unavailable"
    except sr.UnknownValueError:
        # speech was unintelligible
        response["error"] = "Unable to recognize speech"

    return response



class create_impl(object):
    def __init__(self):
        self._lock=threading.Lock()
        self._process = -1
        # self.WORDS = ["stop", "start", "layup", "star", "starred", "thought", "grab there", "letgo",
        #      "freeze", "follow me", "go", "turn", "tug", "reach"]
        self.WORDS = {0:["stop"], 
                      1:["start", "star","Stars", "starred", "thought"], 
                      2:["layup","playoff","they up","play up","layoffs","layout","Leo","they are","Mayotte","leia"],
                      3:["letgo","let go","let's go","Lotto"],
                      4:["wait"],
                      5:["restart"]}
    # create recognizer and mic instances
        mic_list = sr.Microphone.list_microphone_names()
        idx = 3
        print ("Microphone List: ",mic_list,"select: ",mic_list[idx])
        # mic = sr.Microphone(device_index=6)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone(device_index=idx)
        
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration = 3)

        self.updateTestVal = threading.Thread(target=self.run)
        
    def run(self):
            
        while (self._process != 0):
            # with self._lock:    
                
                print('Speak!')
                
                command = recognize_speech_from_mic(self.recognizer, self.microphone)
                
                
        
                # if there was an error, stop the game
                if command["error"]:
                    print("ERROR: {}".format(command["error"]))
                    
                
                else:
                    if not command["transcription"]:            
                        if command["success"]:
                            print("I didn't catch that. What did you say?\n")
                        
                        
                    else:        
                        # show the user the transcription
                        print("You said: {}".format(command["transcription"]))
        
                        # determine if guess is correct and if any attempts remain
                        for idx in range(len(self.WORDS)):
                            if command["transcription"] in self.WORDS[idx]:
                                self._process = idx
                                break
                        # if command["transcription"] in self.WORDS:
                        #     self._process = self.WORDS.index(command["transcription"])
                        #     #self._process = command["transcription"].lower() == self.WORDS[0].lower()
                
                print ("Process:", self._process)
                
                # if self._process == 0:
                #     print ("process ends!")
                #     break        

    def StartStreaming(self):
        x=threading.Thread(target=self.run)
        x.start()
        # thread.start_new_thread(self.run,())
                
    @property        
    def read(self):
        with self._lock:
            return self._process
        

with RR.ServerNodeSetup("experimental.minimal_create", 9999):
    #Register the service type
    RRN.RegisterServiceType(minimal_create_interface)

    create_inst=create_impl()
    
    # # Start running computationally expensive function in parallel
    # updateTestVal = threading.Thread(target=create_inst.run)
    # updateTestVal.setDaemon(True)
    # updateTestVal.start()
    
    #Register the service
    RRN.RegisterService("Create","experimental.minimal_create.create_obj",create_inst)

    #Wait for program exit to quit
    
    input("Press enter to quit \n")
    
