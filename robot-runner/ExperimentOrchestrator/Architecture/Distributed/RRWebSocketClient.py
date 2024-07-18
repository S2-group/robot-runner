import asyncio
import json
import sys
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

    def run_async_process(self, method):
        loop = None
        try:
            loop = asyncio.get_event_loop()
        except:
            loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(method())
    
    def on_message(self, ws, message):
        try:
            data = json.loads(message)
            if data.get("type") == "call":
                callable_method = data.get("method")
                print(f'Method to be called: {callable_method}')
                method = getattr(self, callable_method, None)
                if callable(method):
                    try:
                        self.run_async_process(method)
                    except RuntimeError as rte:
                        print(f'ERROR: {rte.__str__}')
                else:
                    print('Method is not callable!')
        except Exception as e:
            print(f'Something went wrong: {e}')
        ws.send(json.dumps({'type': 'response', 'status': 'completed'}))

    def on_error(self, ws, error):
        sys.exit(0)

    def on_close(self, ws, close_status_code, close_msg):
        print("Connection closed!")

    def on_open(self, ws):
        print("Connection open!")

    def run_forever(self):
        self.ws.run_forever(dispatcher=rel, reconnect=5)
    
