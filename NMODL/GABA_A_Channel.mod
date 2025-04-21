TITLE GABA_A_Channel
COMMENT
A simple GABA_A receptor mechanism that:
 - Uses receptor activation as input
 - Allows chloride (Cl-) current
ENDCOMMENT

NEURON {
    THREADSAFE
    POINT_PROCESS GABA_A_Channel
    USEION cl WRITE icl VALENCE -1  : Define chloride as a negatively charged ion
    RANGE gmax, ecl
    POINTER receptor_activation
}

PARAMETER {
    gmax = 0.1 (uS)    : Maximum conductance
    ecl  = -70 (mV)    : Reversal potential for chloride
}

ASSIGNED {
    v      (mV)        : Membrane potential
    icl    (nA)        : Chloride current
    receptor_activation (1)
}

BREAKPOINT {
    : Chloride current
    icl = gmax * receptor_activation * (v - ecl)
}
