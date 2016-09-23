def gmail_checker(username,password):
  import imaplib,re
  i=imaplib.IMAP4_SSL('imap.gmail.com')
  try:
    i.login(username,password)
    x,y=i.status('INBOX','(MESSAGES UNSEEN)')
    messages=int(re.search('MESSAGES\s+(\d+)',y[0]).group(1))
    unseen=int(re.search('UNSEEN\s+(\d+)',y[0]).group(1))
    return (messages,unseen)
  except:
    return False,0
messages,unseen = gmail_checker('888888','88888')#replace the 88's with your username and password 
if unseen >= 1:
    print "You Got Mail and have"
    print  "you have ",unseen, " Email's"
#else: print "No one likes you"


#print "%i messages, %i unseen" % (messages,unseen)
