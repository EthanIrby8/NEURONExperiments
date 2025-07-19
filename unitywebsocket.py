import socketio
import asyncio
import uvicorn
from neuron_msn_network import NeuronSimulation
from tqdm import tqdm 


sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')
app = socketio.ASGIApp(sio)
connected_clients = set()

@sio.event
async def connect(sid, environ, auth):
    client = sid 
    print(f"Client connected: {client}")
    connected_clients.add(client)
    await send_concentration_data(sid)

@sio.event
async def disconnect(sid, reason):
    client = sid 
    print(f"Client {client} disconnected because {reason}")
    connected_clients.discard(client)

async def send_concentration_data(sid):
    dopamine_concentrations = []
    dop_decay_rates = [0.001, 0.003, 0.005, 0.007, 0.009, 0.01]
    dop_iclamp_amps = [0.2, 0.8, 2.0, 4.0, 6.0, 10.0]
    glutamate_spike_durs = [10.0, 100.0, 200.0, 400.0, 500.0, 700.0]
    for dop_decay_rate, dop_iclamp_amp, glutamate_spike_dur in tqdm(zip(dop_decay_rates, dop_iclamp_amps, glutamate_spike_durs)):
        sim = NeuronSimulation(glutamate_first_spike_time=glutamate_spike_dur, dopamine_first_spike_time=600.0, glutamate_iclamp_amp=0.8, 
                        dopamine_iclamp_amp=dop_iclamp_amp, glutamate_iclamp_delay=8, dopamine_iclamp_delay=4, 
                        glutamate_spike_thresh=-33.0, dopamine_spike_thresh=-33.0, dopamine_decay_rate=dop_decay_rate)
        dop_conc, _ = sim.run_simulation()
        dopamine_concentrations.append(dop_conc)
    sim_dop_conc = list(dopamine_concentrations[2])
    for timestep_dc in sim_dop_conc:
        timestep_dc = round(timestep_dc, 5)
        await sio.emit('nt_concentration', timestep_dc, to=sid)
        await asyncio.sleep(1) 
        

if __name__ == "__main__":
    uvicorn.run(app, host="localhost", port=25001)
