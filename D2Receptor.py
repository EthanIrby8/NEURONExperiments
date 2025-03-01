from neuron import h
import numpy as np 
import logging
from tqdm import tqdm 
from utils import ExponentialSynapse 
logger = logging.getLogger("Logging for D2 Receptor")
FORMAT = '%(asctime)s %(message)s'
logging.basicConfig(format=FORMAT)
logger.setLevel(logging.INFO)




class D2Receptor:
    def __init__(self, exponential_synapse: ExponentialSynapse, resting_membrane_potential: float, G_protein_threshold: float,
                tau: float = 0.15, k: float = 10.46, c: float = 3.62, a: float = 0.09, 
                b: float = 0.012, kV: float = 2.73 * 3600):
        '''models the dynamics of the D2 autoreceptors in the Striatum
        triggers a G-protein cascade effect that decreases dopamine release
        D2AR is a state variable updated at each time step'''
        self.D2AR_prev = 0.0
        self.D2AR = 0.0 # OCCUPIED number of dopamine autoreceptors 
        self.D2AR_total = 0.1 # TOTAL number of dopamine autoreceptors -> fraction 
        self.DAex_prev = 0.0 # concentration of extracellular dopamine in the striatum -> fraction
        self.DAex = 0.0
        self.TDA_prev = 0.0 # pre-synaptic dopamine transporter availability 
        self.TDA = 0.0
        # G-protein state variable 
        self.G = 0.0
        self.G_prev = 0.0
        self.G_protein_threshold = G_protein_threshold 
        self.F_prev = 0.0 # average DA neuron firing rate 
        self.Fmax = 15 * 3600
        self.theta = 25
        self.sigmoid = 18
        self.k = k
        self.kV = kV
        self.a = a 
        self.b = b
        self.c = c
        self.V_rest_prev = resting_membrane_potential
        self.V_rest = 0.0

        self.exponential_synapse = exponential_synapse
        self.nspike = 0
        self.synon = 0
        # variables for synaptic conductance
        self.Ron = 0 
        self.Roff = 0
        self.r0 = 0
        self.t0 = 0

        # turn on multi-order time step integration within NEURON 
        self.cvode = h.CVode()
        self.cvode.active(1)

        # record D2 receptor dynamics 
        self.d2AR_list = []
        self.daEX_list = []
        self.V_list = []
        self.TDA_list = []
        self.G_list = []


    def dD2ARdt(self):
        '''update concentration of activated pre-synaptic D2 receptors'''
        a_unbinding_rate = 1.7
        return self.k * (self.D2AR_total - self.D2AR_prev) * (self.DAex_prev) - a_unbinding_rate * self.D2AR_prev

    def dVOdt(self):
        '''update average resting membrane potential'''
        F = self.Fmax / (1 + np.exp((self.theta - self.V_rest_prev) / self.sigmoid))
        return -self.c * self.V_rest_prev + self.b * F - self.kV * self.D2AR_prev 
 
    def dTDAdt(self):
        '''update pre-synaptic dopamine transporter availability'''
        kT = 87.5
        DO = 0.04
        delta_T = 1.8
        return 1 + (delta_T - 1 / 1 + np.exp(kT * (self.D2AR_prev - DO))) - self.TDA_prev
 
    def dDAexdt(self):
        '''update concentration of extracellular dopamine'''
        F = self.Fmax / (1 + np.exp((self.theta - self.V_rest_prev) / self.sigmoid))
        kVmax = 2.63 * 3600
        kM = 0.2 # affinity of dopamine to dopamine transporter and exhibits minimal spatiotemporal variation 
        beta = 144.0
        return self.a * F - (kVmax * self.TDA_prev / kM + self.DAex_prev) * self.DAex_prev - beta * self.DAex_prev

    def dGdt(self):
        # activation rate of G-proteins as a function of amount of extracellular dopamine 
        k_act = 0.8 
        return k_act * (self.DAex_prev + self.G_prev)

    def set_state_variables(self):
        self.D2AR_prev = self.D2AR 
        self.DAex_prev = self.DAex 
        self.TDA_prev = self.TDA 
        self.V_rest_prev = self.V_rest
        self.G_prev = self.G
        return self.D2AR_prev, self.DAex_prev, self.TDA_prev, self.V_rest_prev, self.G_prev

    def step(self, weight: float, dt: float, total_sim_time: int):
        beta = 0.18
    
        self.nspike += 1

        self.r0 = np.exp(-beta * (dt - self.t0))
        self.t0 = dt
        self.Ron += self.r0
        self.Roff -= self.r0
        self.D2AR += self.dD2ARdt() * dt
        self.DAex += self.dDAexdt() * dt 
        self.V_rest += self.dVOdt() * dt 
        self.TDA += self.dTDAdt() * dt 
        self.G += self.dGdt() * dt
        if self.G > self.G_protein_threshold: 
            logger.info(f"Hyperpolarizing receptor..")
            self.exponential_synapse.synapse.e = -100.0
            logger.info(f"Synapse reversal potential: {self.exponential_synapse.synapse.e}")
        D2AR_prev, DAex_prev, TDA_prev, V_rest_prev, G_prev = self.set_state_variables()
        self.d2AR_list.append(D2AR_prev)
        self.daEX_list.append(DAex_prev)
        self.V_list.append(V_rest_prev)
        self.TDA_list.append(TDA_prev)
        self.G_list.append(G_prev)
        self.synon += weight
        # set synapse conductance (number of channels opened and state of current conducted) to updated weight
        self.exponential_synapse.synapse.g = self.synon * (self.Ron + self.Roff)

        if dt < total_sim_time:
            self.cvode.event(dt + 0.3, lambda: self.step(weight, dt + 0.3, total_sim_time))

        # logger.info(f"G-protein activation hist: {self.G_list}")
        # logger.info(f"DA extracellular concentration hist: {self.daEX_list}")

        return self.d2AR_list, self.daEX_list, self.V_list, self.TDA_list, self.G_list


    def deactivate(self, flag: int, weight: float, t: float):
        Rinf = 0.94 / (0.94 + 0.18)
        Rtau = 1 / (0.94 + 0.18)

        if flag == self.nspike:
            self.r0 = weight*Rinf + (self.r0 - weight*Rinf) + np.exp(-(t - self.t0) / Rtau) 
            self.t0 = t
            self.synon -= weight
            self.Ron -= self.r0
            self.Roff += self.r0