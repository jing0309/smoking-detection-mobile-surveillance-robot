#import tkinter as tk
from fpdf import FPDF
from datetime import datetime
from pytz import timezone

import tkinter as tk
import tkinter.messagebox as msg

from gtts import gTTS
import os

def user_input():
  root=tk.Tk()
  root.title("Please Enter Your Information")
  root.geometry("850x250+650+300")

  name_var=tk.StringVar()
  id_var=tk.StringVar()

  def submit():
      text = "Thank you for your coorperation. Your information have been recorded."
    
      tts = gTTS(text) # Speech Synthesis
      tts.save("speech.mp3")
      os.system("mpg321 speech.mp3")
      os.remove("speech.mp3")

      msg.showinfo("Notice", text) # Pop Up Message
  
      # name = name_var.get()
      # id_ic = id_var.get()
      
      # print("Name : " + name)
      # print("ID/IC : " + id_ic)
      root.destroy()

  name_label = tk.Label(root, text = '  Name: ', font=('Times',15, 'bold'))
  name_entry = tk.Entry(root,textvariable = name_var, font=('Times',15,'normal'),width=30)
  ID_label = tk.Label(root, text = '  ID/IC: ', font = ('Times',15,'bold'))
  ID_entry=tk.Entry(root, textvariable = id_var, font = ('Times',15,'normal'),width=30)
  sub_btn=tk.Button(root,text = 'Submit', command = submit, font = ('Times',15,'bold'))
    
  name_label.grid(row=1,column=1)
  name_entry.grid(row=1,column=2)
  ID_label.grid(row=3,column=1)
  ID_entry.grid(row=3,column=2)
  sub_btn.grid(row=5,column=2)

  root.mainloop()
  name = name_var.get()
  id_ic = id_var.get()

  return name,id_ic

  # name = str(input("Please enter your name: "))
  # id_ic = input("Please enter your student/staff id_ic: ")
  # return name,id_ic

def createpdf():
  name, id_ic = user_input() # Get Name and ID/IC
  
  f = open("/home/mustar/catkin_ws/src/fyp_jiamun/fyp/identity.txt", "r") # Get Identity
  identity = f.read()
  f.close()
  
  # need to change path, refer the file wrote by navigation
  g = open("/home/mustar/catkin_ws/src/fyp_jiamun/fyp/location.txt", "r") # Get location
  location = g.read()
  g.close()

  pdf = FPDF() #pdf = FPDF('P', 'mm', 'A4') [Portrait, A4, measure unit: millimeter]
  pdf.add_page()
  pdf.set_margins(left=30, top=float, right= float)
  pdf.set_font('Arial', 'BU', 16)
  pdf.cell(w=190, h = 16, txt = 'SMOKER DETECTION REPORT', border = 0,ln = 2, align = 'C', fill = False, link = '')
  pdf.image("/home/mustar/catkin_ws/src/fyp_jiamun/fyp/Smoker.jpg", x = 70, y = 35, w = 70, h = 0, type = 'JPG', link = '') # need to change path, refer image processing module, see where it stores the image captured
  pdf.set_font('Arial', 'B', 13)
  pdf.text(x=30,y=100,txt = 'Name: '+ name)
  pdf.text(x=30,y=110,txt = 'ID: '+ id_ic)
  pdf.text(x=30,y=120,txt = 'Identity: '+ identity)
  #pdf.text(x=30,y=160,txt = 'Gender: ')
  pdf.text(x=30,y=130,txt = 'Location Detected: ' + location)
  now = datetime.now(timezone('Asia/Kuala_Lumpur'))
  format = "%Y-%m-%d %H:%M:%S"
  current_time = now.strftime(format)
  print(current_time)
  pdf.text(x=30,y=140,txt = 'Date & Time: ' + current_time)
  pdf.output('/home/mustar/catkin_ws/src/fyp_jiamun/fyp/Smoker_Detection_Report.pdf', 'F')
  return name

def prompt_input():
  f = open("/home/mustar/catkin_ws/src/fyp_jiamun/fyp/identity.txt", "r") # Get Identity
  identity = f.read()
  f.close()
  text = ""
  # synthesis speech based on rnn result
  if identity == "UM Staff":
    text = "Please insert your name and Staff ID"
  elif identity == "UM Student":
    text = "Please insert your name and Student ID"
  else:
    text = "Please insert your name and Identification Card number. If you don't have Identification Card, please insert your passport number."
  
  tts = gTTS(text) # Speech Synthesis
  tts.save("speech.mp3")
  os.system("mpg321 speech.mp3")
  os.remove("speech.mp3")


def announcement(name):
  # give gentle reminder
  text = '''Hello {x}, I am responsible to advise you to not smoking in this area. 
  Students need a clean and healthy environment to study. Thank you and wish you a nice day.'''.format(x=name)
    
  tts = gTTS(text) # Speech Synthesis
  tts.save("speech.mp3")
  os.system("mpg321 speech.mp3")
  os.remove("speech.mp3")


if __name__ == "__main__":
  prompt_input()
  name = createpdf()
  announcement(name)




