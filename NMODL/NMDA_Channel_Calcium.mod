TITLE NMDA Receptor with Calcium Dynamics

NEURON {
    THREADSAFE
    POINT_PROCESS NMDA_Channel_Calcium
    NONSPECIFIC_CURRENT i
    RANGE gmax, e, mgblock, mg, alpha_mg, beta_mg, depth, taur, local_cai
    POINTER receptor_activation
}

CONSTANT {
    FARADAY = 96485.3 (coulomb/mol) : Faraday's constant
}

PARAMETER {
    gmax = 0.1 (uS)          : Maximum total conductance
    e = 0 (mV)               : Reversal potential
    depth = 0.1 (um)         : Shell depth for calcium accumulation
    taur = 200 (ms)          : Calcium decay time constant
    mg = 1 (mM)              : External [Mg2+]
    alpha_mg = 0.062 (/mV)   : Voltage-dependence factor
    beta_mg = 3.57           : Mg2+ block scaling
}

ASSIGNED {
    v (mV)                   : Membrane potential
    i (nA)                   : Nonspecific current
    g (uS)                   : Total channel conductance
    mgblock                  : Magnesium block factor
    drive_ca (mM/ms)         : Calcium influx rate
    receptor_activation (1)
}

STATE {
    local_cai (mM)           : Localized calcium concentration
}


BREAKPOINT {
    SOLVE state METHOD cnexp

    : Calculate Mg2+ block
    mgblock = 1 / (1 + mg * beta_mg * exp(-alpha_mg * v))

    : Total conductance
    g = gmax * receptor_activation * mgblock

    : Nonspecific current
    i = g * (v - e)

    : Calcium influx rate
    drive_ca = -g * (v - 140) / (2 * FARADAY * depth)

    : Prevent unphysical calcium extrusion
    if (drive_ca <= 0) {
        drive_ca = 0
    }
}

DERIVATIVE state {
    : Localized calcium dynamics
    local_cai' = drive_ca - local_cai / taur
}