TITLE GABA_A_Channel
COMMENT
A simple GABA_A receptor mechanism that:
 - Uses receptor activation as input
 - Allows chloride (Cl-) current
ENDCOMMENT

NEURON {
    POINT_PROCESS GABA_A_Channel
    USEION cl WRITE icl VALENCE -1  : Define chloride as a negatively charged ion
    RANGE gmax, ecl
    POINTER receptor_activation
}

PARAMETER {
    gmax = 0.1 (uS)    : Maximum conductance
    ecl  = -70 (mV)    : Reversal potential for chloride
}

INITIAL {
    receptor_activation = 1
}

ASSIGNED {
    v      (mV)        : Membrane potential
    receptor_activation (1)
    icl    (nA)        : Chloride current
}

BREAKPOINT {
    : Chloride current
    icl = gmax * receptor_activation * (v - ecl)
}
