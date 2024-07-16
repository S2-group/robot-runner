import asyncio
from ExperimentOrchestrator.Architecture.Distributed.RRWebSocketClient import RRWebSocketClient

class RRClient(RRWebSocketClient):
    
    # overrides before_experiment from abstract class
    def remote_method():
        print('Running before experiment remotelly!')

        # Your code here

async def main():
    server_address = "ws://server.robot-runner:8765"
    client_id = "c1"
    client = RRClient(server_address, client_id)
    await client.register()

try:
    loop = asyncio.get_running_loop()
except RuntimeError:
    asyncio.run(main())
else:
    loop.create_task(main())
