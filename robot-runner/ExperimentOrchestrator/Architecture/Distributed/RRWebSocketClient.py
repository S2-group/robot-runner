import json
import websocket
from websocket import create_connection
import rel

class RRWebSocketClient:

    def __init__(self, server_address):
        self.server_address = server_address
        self.kill_signal_received = False    
        
        self.ws = websocket.WebSocketApp(server_address,
                                         on_open=self.on_open,
                                         on_message=self.on_message,
                                         on_error=self.on_error,
                                         on_close=self.on_close)
                
    def register(self, client_id):
        registration_data = {
                "type": "register",
                "client_id": client_id  
            }
        self.ws.send(json.dumps(registration_data))
    
    def on_message(self, ws, message):
        try:
            data = json.loads(message)
            if data.get("type") == "call":
                callable_method = data.get("method")
                print(f'Method to be called: {callable_method}')
                method = getattr(self, callable_method, None)
                if callable(method):
                    method()
                else:
                    print('Method is not callable!')
        except:
            print('Something went wrong!')

    def on_error(self, ws, error):
        print(error)

    def on_close(self, ws, close_status_code, close_msg):
        print("Connection closed!")

    def on_open(self, ws):
        print("Connection open!")

    def run_forever(self):
        self.ws.run_forever(dispatcher=rel, reconnect=5)

    def clean_docker(self):
        print('Cleaning Docker!')
        # Your code here
    
