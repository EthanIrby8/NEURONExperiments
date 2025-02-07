from neuron import h
from tqdm import tqdm 
import matplotlib.pyplot as plt 
import numpy as np 



class SynapticNeuron:
    def __init__(self, neuron_type: str, gid: int, hodgkin_huxley: bool, create_dendrite: bool, 
                x: float, y: float, z: float, theta: float, soma_length: float = 12.6157, soma_diameter: float = 12.6157, 
                n_soma_segments: int = 1, n_dendrite_segments: int = 1,
                axon_length: int = 1000, axon_diameter: int = 5, dendrite_length: int = 200, dendrite_diameter: int = 1):

            self._gid = gid
            self.neuron_type = neuron_type
            self.hodgkin_huxley = hodgkin_huxley
            self.create_dendrite = create_dendrite
            self.soma_length = soma_length
            self.soma_diameter = soma_diameter
            self.axon_length = axon_length
            self.axon_diameter = axon_diameter
            self.dendrite_length = dendrite_length
            self.dendrite_diameter = dendrite_diameter
            self.n_soma_segments = n_soma_segments
            self.n_dendrite_segments = n_dendrite_segments
            self.build_morphology()
            self.all = self.soma.wholetree()
            self.x, self.y, self.z = 0, 0, 0
            h.define_shape()
            self._rotate_z(theta)
            self._set_position(x, y, z)

            # record spike times, membrane potential over time, and network connection between a pre and post neuron 
            ## creating a network connection starting from the current neuron without considering an external input source
            self._spike_detector = h.NetCon(self.soma(0.5)._ref_v, None, sec=self.soma)
            self.spike_times = h.Vector()
            self._spike_detector.record(self.spike_times)
            self.ncs = []
            self.soma_v = h.Vector().record(self.soma(0.5)._ref_v)
            

    def build_morphology(self):
        self.mV = 0.02
        self.soma = h.Section(name=self.neuron_type, cell=self)
        self.soma.L = self.soma_length 
        self.soma.diam = self.soma_diameter
        self.soma.nseg = self.n_soma_segments

        ### AXONS
        # self.axon = h.Section(name="axon", cell=self)
        # self.axon.L = self.axon_length 
        # self.axon.diam = self.axon_diameter

        # Connect the axon to the end of the soma
        # self.axon.connect(self.soma(1))

        if self.create_dendrite:
            self.dendrites = h.Section(name="dendrites", cell=self)
            self.dendrites.L = self.dendrite_length 
            self.dendrites.diam = self.dendrite_diameter
            self.dendrites.nseg = self.n_dendrite_segments

            # Connect dendrite to the start of the soma
            self.dendrites.connect(self.soma)

            self.dendrites.insert("pas")
            for seg in self.dendrites:
                seg.pas.g = 0.001 # Passive conductance (how far the membrane potential is driven by the input current)
                seg.pas.e = -65 * self.mV # Reversal potential - the membrane potential where the direction of the current reverses
            
            self.syn = ExponentialSynapse(self, soma_synapse=False)
            self.syn.synapse.tau = 2
            
        if self.hodgkin_huxley:
            self.mV = 0.02
            self.build_hh(self.mV)

        for sec in self.soma.wholetree():
            sec.Ra = 100  # Axial resistance in Ohm * cm
            sec.cm = 1
        
    def build_hh(self, mV):
        self.soma.insert("hh")
        for segment in self.soma:
            segment.hh.gnabar = 0.12 # sodium conductance
            segment.hh.gkbar = 0.036 # potassium conductance
            segment.hh.gl = 0.003 # leak conductance
            segment.hh.el = -54.3 * mV # reversal potential -> TODO: MODULATE

    def __repr__(self):
        return f"SynapticNeuron[{self._gid}]"

    def _set_position(self, x, y, z):
        for sec in self.all:
            for i in range(sec.n3d()):
                sec.pt3dchange(i, x - self.x + sec.x3d(i), y - self.y + sec.y3d(i), 
                z - self.z + sec.z3d(i), sec.diam3d(i))
        self.x, self.y, self.z = x, y, z

    def _rotate_z(self, theta):
        """Rotate the neuron wrapper about the Z axis."""
        for sec in self.all:
            for i in range(sec.n3d()):
                x = sec.x3d(i)
                y = sec.y3d(i)
                c = h.cos(theta)
                s = h.sin(theta)
                xprime = x * c - y * s
                yprime = x * s + y * c
                sec.pt3dchange(i, xprime, yprime, sec.z3d(i), sec.diam3d(i))


class ExponentialSynapse:
    def __init__(self, neuron: SynapticNeuron, soma_synapse: bool, position: float = 0.5, tau: float = 2.0, 
                reversal_potential: float = 0.0):
        if soma_synapse:
            self.synapse = h.ExpSyn(neuron.soma(position))
        else:
            self.synapse = h.ExpSyn(neuron.dendrites(position))
        self.synapse.tau = tau # decay time
        self.synapse.e = reversal_potential
        self.neuron = neuron


class InputCurrent:
    def __init__(self, average_number_spikes: int = 20, first_spike_time: float = 3.0, 
                average_time_between_spikes: float=5.0):

        # Generate presynaptic stimuli
        self.input_current = h.NetStim() 
        self.input_current.number = average_number_spikes
        self.input_current.start = first_spike_time
        self.input_current.interval = average_time_between_spikes