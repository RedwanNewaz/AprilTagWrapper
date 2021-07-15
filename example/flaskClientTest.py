import requests
import time
import json
'''
To use this example install requests using
pip3 install requests
'''

if __name__ == '__main__':
    for _ in range(10):
        time.sleep(1)
        url = "http://127.0.0.1:5000"
        data = requests.get(url).text
        jdata = json.dumps(data, ensure_ascii=False)
        value = json.loads(jdata)
        print(value)