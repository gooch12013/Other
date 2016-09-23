import serial
import time


#else: print "No one likes you"



DMXOPEN = chr(126)
DMXCLOSE = chr(231)
DMXINTENSITY = chr(6) + chr(1) + chr(2)
DMXINIT1 = chr(03) + chr(02) + chr(0) + chr(0) + chr(0)
DMXINIT2 = chr(10) + chr(02) + chr(0) + chr(0) + chr(0)
ser = serial.Serial(0)
ser.write(DMXOPEN + DMXINIT1 + DMXCLOSE)
ser.write(DMXOPEN + DMXINIT2 + DMXCLOSE)
dmxdata = [chr(0)] * 513


def senddmx(data, chan, intensity):
      data[chan] = chr(intensity)
      sdata = ''.join(data)
      ser.write(DMXOPEN + DMXINTENSITY + sdata + DMXCLOSE)
      return (data)

#print "%i messages, %i unseen" % (messages,unseen)

x = 1
while True:

    def gmail_checker(username, password):
        import imaplib, re
        i = imaplib.IMAP4_SSL('imap.gmail.com')
        try:
            i.login(username, password)
            x, y = i.status('INBOX', '(MESSAGES UNSEEN)')
            messages = int(re.search('MESSAGES\s+(\d+)', y[0]).group(1))
            unseen = int(re.search('UNSEEN\s+(\d+)', y[0]).group(1))
            return (messages, unseen)
        except:
            return False, 0


    messages, unseen = gmail_checker('*******', '*****')
    if unseen >= 1:
        #print "You Got Mail and have"
        #print  "you have ", unseen, " Email's"
        dmxdata = senddmx(dmxdata, 1, 255)
        dmxdata = senddmx(dmxdata, 8, 255)
        dmxdata = senddmx(dmxdata, 6, 200)
    else:
            dmxdata = senddmx(dmxdata, 1, 0)

x + 1
