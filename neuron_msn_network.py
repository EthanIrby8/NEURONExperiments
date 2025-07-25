from neuron import h

h.load_file("stdrun.hoc")
h.nrn_load_dll("./NMODL/arm64/.libs/libnrnmech.so")
from utils import InputCurrent
import numpy as np
import matplotlib.pyplot as plt
import logging

logging.basicConfig(format="%(asctime)s %(message)s", filemode="w")
logger = logging.getLogger()
logger.setLevel(logging.INFO)

RESTING_MEMBRANE_POTENTIAL = -65.0
TOTAL_EXP_TIME = 1200


class NeuronSimulation:
    def __init__(
        self,
        glutamate_first_spike_time,
        dopamine_first_spike_time,
        glutamate_iclamp_amp,
        dopamine_iclamp_amp,
        glutamate_iclamp_delay,
        dopamine_iclamp_delay,
        glutamate_spike_thresh,
        dopamine_spike_thresh,
        dopamine_decay_rate,
    ):
        self.glutamate_first_spike_time = glutamate_first_spike_time
        self.dopamine_first_spike_time = dopamine_first_spike_time
        self.glutamate_iclamp_amp = glutamate_iclamp_amp
        self.dopamine_iclamp_amp = dopamine_iclamp_amp
        self.glutamate_iclamp_delay = glutamate_iclamp_delay
        self.dopamine_iclamp_delay = dopamine_iclamp_delay
        self.glutamate_spike_thresh = glutamate_spike_thresh
        self.dopamine_spike_thresh = dopamine_spike_thresh
        self.dopamine_decay_rate = dopamine_decay_rate

    def glutamate_init(
        self,
        syn_connection_weight=150.0,
        init_concentration=0.8,
        spike_delay=1,
    ):
        glutamate_soma = h.Section("glutamate_soma")
        glutamate_dendrites = h.Section("glutamate_dendrites")
        glutamate_dendrites.connect(glutamate_soma(1))
        for sec in [glutamate_soma, glutamate_dendrites]:
            sec.insert("hh")
        for seg in glutamate_soma:
            seg.hh.gnabar = 0.12
            seg.hh.gkbar = 0.036
            seg.hh.gl = 0.0003
            seg.hh.el = -20.0
        # initial current to glutamate soma
        glutamate_iclamp = h.IClamp(glutamate_soma(0.5))
        glutamate_iclamp.delay = self.glutamate_iclamp_delay
        glutamate_iclamp.dur = self.glutamate_first_spike_time
        glutamate_iclamp.amp = self.glutamate_iclamp_amp

        glutamate_syn = h.ExpSyn(glutamate_soma(1))
        glutamate_syn.e = self.glutamate_spike_thresh
        glutamate_syn.tau = 0.01

        glutamate_netcon = h.NetCon(
            glutamate_soma(1)._ref_v, glutamate_syn, sec=glutamate_soma
        )
        glutamate_netcon.threshold = -20.0
        glutamate_netcon.weight[0] = syn_connection_weight
        glutamate_netcon.delay = spike_delay

        return glutamate_soma, glutamate_syn

    def dopamine_init(
        self,
        init_concentration=0.9,
        syn_connection_weight=50.0,
    ):
        dopamine_soma = h.Section("dopamine_soma")
        dopamine_dendrites = h.Section("dopamine_dendrites")
        dopamine_dendrites.connect(dopamine_soma(1))
        for sec in [dopamine_soma, dopamine_dendrites]:
            sec.insert("hh")
        for seg in dopamine_soma:
            seg.hh.gnabar = 0.12
            seg.hh.gkbar = 0.036
            seg.hh.gl = 0.003
            seg.hh.el = -20.0
        # initial current to dopamine soma
        dopamine_iclamp = h.IClamp(dopamine_soma(0.5))
        dopamine_iclamp.delay = self.dopamine_iclamp_delay
        dopamine_iclamp.dur = self.dopamine_first_spike_time
        dopamine_iclamp.amp = self.dopamine_iclamp_amp
        # TODO: add mod file for d2 receptor. model dopamine concentration with more dynamics
        dopamine_neuron = h.GenericLigand(dopamine_soma(0.5))
        dopamine_neuron.C_init = init_concentration
        dopamine_neuron.decay_rate = self.dopamine_decay_rate

        dopamine_syn = h.ExpSyn(dopamine_soma(1))
        dopamine_syn.e = self.dopamine_spike_thresh

        return dopamine_soma, dopamine_neuron, dopamine_syn

    def msn_init(
        self,
        num_ligands=2,
        max_receptor_binding_capacity=1.0,
        spike_thresh=0.0,
        syn_connection_weight=150.0,
        spike_delay=1,
    ):
        # medium spiny neuron in dorsal striatum with gabaergic properties
        # TODO: have multiple receptors -> 1. glutamate (AMPA/NMDA) 2. dopamine (D2) 3. gaba (GABA-A/GABA-B)
        msn_soma = h.Section("msn_soma")
        msn_dendrites = h.Section("msn_dendrites")
        msn_dendrites.connect(msn_soma(1))
        for sec in [msn_soma, msn_dendrites]:
            sec.insert("hh_prophos")
        for seg in msn_soma:
            seg.hh_prophos.gnabar = 0.12
            seg.hh_prophos.gkbar = 0.036
            seg.hh_prophos.gl = 0.0003
            seg.hh_prophos.el = -20.0
        # medium spiny neuron in dorsal striatum that receives glutamate input from the cerebral cortex and dopamine from the SNc
        msn_neuron = h.GenericLigand(msn_soma(0.5))
        # msn synapse
        msn_syn = h.ExpSyn(msn_soma(1))
        msn_syn.e = spike_thresh

        msn_dopamine_receptor = h.GenericReceptor(msn_soma(1))
        msn_dopamine_receptor.n_ligands = num_ligands
        msn_dopamine_receptor.capacity = max_receptor_binding_capacity

        return (
            msn_soma,
            msn_neuron,
            msn_syn,
            msn_dopamine_receptor,
        )

    def create_netcons(
        self,
        glutamate_soma,
        msn_synapse,
        dopamine_soma,
        msn_neuron,
        syn_connection_weight=150.0,
        spike_thresh=0.0,
        spike_delay=1,
    ):

        # connect glutamate voltage to msn synapse
        glutamate_msn_soma_netcon = h.NetCon(
            glutamate_soma(0.5)._ref_v, msn_synapse, sec=glutamate_soma
        )
        glutamate_msn_soma_netcon.threshold = spike_thresh
        glutamate_msn_soma_netcon.weight[0] = syn_connection_weight
        glutamate_msn_soma_netcon.delay = spike_delay

        # connect dopamine voltage to msn synapse
        dopamine_msn_soma_netcon = h.NetCon(
            dopamine_soma(0.5)._ref_v, msn_synapse, sec=dopamine_soma
        )
        dopamine_msn_soma_netcon.threshold = spike_thresh
        dopamine_msn_soma_netcon.weight[0] = syn_connection_weight
        dopamine_msn_soma_netcon.delay = spike_delay

        # connect glutamate voltage to msn concentration
        glutamate_msn_netcon = h.NetCon(
            glutamate_soma(0.5)._ref_v, msn_neuron, sec=glutamate_soma
        )
        glutamate_msn_netcon.threshold = spike_thresh
        glutamate_msn_netcon.weight[0] = syn_connection_weight
        glutamate_msn_netcon.delay = spike_delay
        logger.info("Created netcons")

    def record_network_data(
        self,
        glutamate_soma,
        dopamine_soma,
        msn_soma,
        dopamine_neuron,
        msn_neuron,
    ):
        t_vec = h.Vector().record(h._ref_t)
        v_glutamate = h.Vector().record(glutamate_soma(0.5)._ref_v)
        v_dopamine = h.Vector().record(dopamine_soma(0.5)._ref_v)
        v_msn = h.Vector().record(msn_soma(1)._ref_v)
        dopamine_conc = h.Vector().record(dopamine_neuron._ref_C)
        msn_conc = h.Vector().record(msn_neuron._ref_C)
        gk_record = h.Vector().record(msn_soma(0.5).hh_prophos._ref_gk)

        return (
            msn_conc,
            dopamine_conc,
            t_vec,
            v_msn,
            v_dopamine,
            v_glutamate,
            gk_record,
        )

    def run_simulation(self):
        # Create single glutamate, dopamine, and medium spiny neurons
        glu_soma, glu_syn = self.glutamate_init(
            syn_connection_weight=150.0,
            init_concentration=0.8,
            spike_delay=1,
        )

        dop_soma, dop_neuron, dop_syn = self.dopamine_init(
            init_concentration=0.9,
            syn_connection_weight=50.0,
        )

        msn_soma, msn_neuron, msn_syn, msn_dop_receptor = self.msn_init(
            num_ligands=1,
            max_receptor_binding_capacity=1.0,
            spike_thresh=-33.0,
            syn_connection_weight=1.0,
            spike_delay=1,
        )

        # set the ligand concentration to each unoccupied ligand binding site of the MSN receptor
        h.setpointer(dop_neuron._ref_C, "C_lig1", msn_dop_receptor)
        h.setpointer(
            msn_dop_receptor._ref_activation,
            "receptor_activation",
            msn_soma(0.5).hh_prophos,
        )

        self.create_netcons(
            glu_soma,
            msn_syn,
            dop_soma,
            msn_neuron,
            syn_connection_weight=100.0,
            spike_thresh=-20.0,
            spike_delay=1,
        )

        (
            msn_concentration,
            dopamine_concentration,
            time_vector,
            voltage_msn,
            dopamine_volt,
            glutamate_v,
            gk_record,
        ) = self.record_network_data(
            glu_soma, dop_soma, msn_soma, dop_neuron, msn_neuron
        )

        h.finitialize(RESTING_MEMBRANE_POTENTIAL)
        h.continuerun(TOTAL_EXP_TIME)
        logger.info("Ran simulation")
        return dopamine_concentration, time_vector, gk_record, voltage_msn


sim = NeuronSimulation(
    glutamate_first_spike_time=500.0,
    dopamine_first_spike_time=600.0,
    glutamate_iclamp_amp=0.8,
    dopamine_iclamp_amp=0.4,
    glutamate_iclamp_delay=8,
    dopamine_iclamp_delay=4,
    glutamate_spike_thresh=-33.0,
    dopamine_spike_thresh=-33.0,
    dopamine_decay_rate=0.004,
)
"""
dopamine_concentration, time_vector, gk_record, voltage_msn = sim.run_simulation()

plt.figure(figsize=(12, 6))
plt.plot(time_vector, voltage_msn, label="V")
plt.xlabel("Time (ms)")
plt.ylabel("Voltage (mV)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure(figsize=(12, 6))
plt.plot(time_vector, gk_record, label="gK")
plt.xlabel("Time (ms)")
plt.ylabel("gK (S/cm^2)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure(figsize=(12, 6))
plt.plot(
    time_vector,
    dopamine_concentration,
    label="Dopamine concentration in the Dorsal Striatum",
)
plt.xlabel("Time (ms)")
plt.ylabel("Concentration (uM)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
"""
