import tensorflow as tf
import pickle # pickle5                       0.0.12
from tensorflow.keras.preprocessing.sequence import pad_sequences
import pandas as pd
import tkinter as tk

from gtts import gTTS
import os

max_length = 50

s_model = tf.keras.models.load_model("/home/mustar/catkin_ws/src/fyp_jiamun/fyp/student_model")
with open('/home/mustar/catkin_ws/src/fyp_jiamun/fyp/student_model/tokenizer.pkl', 'rb') as input:
    tokenizer = pickle.load(input)

f = open("/home/mustar/catkin_ws/src/fyp_jiamun/fyp/respond.txt", "r")
read = f.read()
f.close()
sms = [read]
# sms = ["i am a student from this faculty"]
# sms = ["i am a lecturer"]
# sms = ["hello! how are you? im visiting mom next week"]
sms_proc = tokenizer.texts_to_sequences(sms)
sms_proc = pad_sequences(sms_proc, maxlen=max_length, padding='post')
pred = s_model.predict(sms_proc)

#find max
df = pd.DataFrame(pred)
max = pred[0][0]
preds = 0
for i in range(3):
    if pred[0][i]>max:
        max = pred[0][i]
        preds = i
print(preds)

def identity_update(text):
    global identity
    identity = text
    root.quit()
    return identity

if preds == 0:
    identity = "UM Staff"
elif preds == 1: 
    identity = "UM Student"
elif preds == 2:

    text = "I could not identify your identity. Please select your identity."
    
    tts = gTTS(text) # Speech Synthesis
    tts.save("speech.mp3")
    os.system("mpg321 speech.mp3")
    os.remove("speech.mp3")

    root = tk.Tk()
    root.title("Please Select Your Identity")
    root.geometry("600x250+900+300")

    button_dict={}
    option= ["UM Staff", "UM Student", "Guest"]

    for i in option:
        def func(x=i):
            return identity_update(x)

        button_dict[i]=tk.Button(root, text=i, command= func,font = ('Times',15,'bold'))
        button_dict[i].pack()
    
    root.mainloop()

f = open("/home/mustar/catkin_ws/src/fyp_jiamun/fyp/identity.txt", "w")
f.write(identity)
f.close()







