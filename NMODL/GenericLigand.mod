TITLE Modular Ligand Class Supporting External Input

NEURON {
    THREADSAFE
    POINT_PROCESS GenericLigand
    RANGE C_init, decay_rate, external_input, C, receptor_activation
}

PARAMETER {
    C_init = 0 (uM)           : Initial ligand concentration
    decay_rate = 0.01 (/ms)   : Decay rate for ligand decay
    external_input = 0 (uM/ms): Continuous environmental input (volume transmission)
}

STATE {
    C (uM)                    : Ligand concentration (dynamic)
}

INITIAL {
    C = C_init                : Initialize C to its starting value
}

ASSIGNED {
    receptor_activation (1)
}

BREAKPOINT {
    SOLVE state METHOD cnexp
}

DERIVATIVE state {
    : Ligand evolves through decay and external input
    C' = -C * decay_rate + external_input
}

NET_RECEIVE(weight (uM)) {
    : Increment ligand concentration based on incoming spikes
    C = C + weight
}
