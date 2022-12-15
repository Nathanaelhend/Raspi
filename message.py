import time
import requests
import telepot
from telepot.loop import MessageLoop


# def handle(msg):
#     chat_id = msg['chat']['id'] # Receiving the message from telegram
#     command = msg['text']   # Getting text from the message

#     print ('Received:')
#     print(command)

#     # Comparing the incoming message to send a reply according to it
#     bot.sendMessage (chat_id, str("Hi! bot test_eye"))
    

# Insert your telegram token below
# bot = telepot.Bot('5513926646:AAEWbyK6_AxMWLyAElwWfoDpZ0lAnzPdl3E')
# print (bot.getMe())
while True:
    url = 'https://api.telegram.org/bot5513926646:AAEWbyK6_AxMWLyAElwWfoDpZ0lAnzPdl3E/sendMessage?chat_id=922321291&text="Sopir Mengantuk!"'
    requests.get(url)

    time.sleep(5)

# Start listening to the telegram bot and whenever a message is  received, the handle function will be called.
# MessageLoop(bot, handle).run_as_thread()
# print ('Listening....')


# while 1:
#     time.sleep(10)