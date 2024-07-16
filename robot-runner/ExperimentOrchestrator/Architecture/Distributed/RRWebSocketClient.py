from abc import abstractmethod
import sys
import websockets
import json

class RRWebSocketClient:
    def __init__(self, server_address, client_id):
        self.server_address = server_address
        self.client_id = client_id
        self.kill_signal_received = False
        
    async def register(self):
        async with websockets.connect(self.server_address) as websocket:
            registration_data = {
                "type": "register",
                "client_id": self.client_id
            }
            await websocket.send(json.dumps(registration_data))
            while not self.kill_signal_received:
                try:
                    message = await websocket.recv()
                    try:
                        if message == "kill":
                            self.kill_signal_received = True
                            print("Received kill signal. Terminating client.")
                        else:
                            data = json.loads(message)
                            if data.get("type") == "call":
                                callable_method = data.get("method")
                                print(f'Method to be called: {callable_method}')
                                # Call Method
                                #
                                #
                            else:
                                print("Invalid request! Nothing to do!")
                    except json.JSONDecodeError as jsonerror:
                            print("Invalid request or JSON format issue.")
                            print(message)
                except websockets.exceptions.ConnectionClosedError as conerror:
                    print("Connection closed by the server!")
                    sys.exit(0)
                    
