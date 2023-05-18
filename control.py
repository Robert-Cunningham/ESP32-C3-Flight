import requests

while True:
    throttle = input("Enter throttle: ")
    roll = input("Enter roll: ")
    pitch = input("Enter pitch: ")
    yaw = input("Enter yaw: ")

    command = {'throttle': throttle, 'roll': roll, 'pitch': pitch, 'yaw': yaw}

    response = requests.post("http://192.168.4.1/command", params=command)
    print("Response: ", response.text)