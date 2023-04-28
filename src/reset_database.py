from firebase import firebase

# Read the url of database from txt file 
with open(r"database_url.txt","r") as urlFile:
    url = urlFile.read()

appHandle = firebase.FirebaseApplication(dsn=url, authentication=None)

appHandle.put(url='', name='availability', data='yes')
appHandle.put(url='currentDelivery', name='status', data='none')

appHandle.put(url='currentDelivery/Receiver', name='Location', data="nil")
appHandle.put(url='currentDelivery/Sender', name='Location', data="nil")

appHandle.put(url='currentDelivery/Receiver', name='Name', data="nil")
appHandle.put(url='currentDelivery/Sender', name='Name', data="nil")

print("Reset successful")
