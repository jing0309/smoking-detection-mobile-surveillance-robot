#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import speech_recognition as sr
from gtts import gTTS
import os


def googlesr():
    rospy.init_node('googlesr', anonymous=True)
    pub = rospy.Publisher('input', String, queue_size=10)
    text = "Hi, I am a surveillance robot in this university. I have detected you are smoking in non-smoking area. "
    tts = gTTS(text)
    tts.save("speech.mp3")
    os.system("mpg321 speech.mp3")
    os.remove("speech.mp3")

    while not rospy.is_shutdown():
        
        text = "Please tell me your identity. Are you a student or staff from this university?"
        tts = gTTS(text)
        tts.save("speech.mp3")
        os.system("mpg321 speech.mp3")
        os.remove("speech.mp3")
        
        # obtain audio from the microphone
        r = sr.Recognizer()
        
        with sr.Microphone() as source:
            print(text)
            #audio = r.listen(source)
            audio = r.record(source, duration=5)
            
        # recognize speech using Google Speech Recognition
        try:
            result = r.recognize_google(audio)
            print("SR result: " + result)
            f = open("/home/mustar/catkin_ws/src/fyp_jiamun/fyp/respond.txt", "w")
            f.write(result)
            f.close()

            pub.publish(result)
            rospy.signal_shutdown("done")

        except sr.UnknownValueError:
            print("SR could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
        

if __name__ == '__main__':
    try:
        googlesr()
    except rospy.ROSInterruptException:
        pass

