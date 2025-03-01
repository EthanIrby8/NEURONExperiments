NEURON {
  THREADSAFE
  POINT_PROCESS SynNoise
  RANGE i,del,dur,f0,f1,r,torn,std,bias, noise
  ELECTRODE_CURRENT i
}

VERBATIM
extern void set_seed(double);
extern double normrand(double, double);
ENDVERBATIM

UNITS {
  (nA) = (nanoamp)
}

PARAMETER {
  del=50    (ms)
  dur=200   (ms)
  torn=500  (ms)
  std=0.2   (nA)
  f0=0.001    (nA)
  f1=0.002    (nA)
  r =60
  bias = 0 (nA)
}

ASSIGNED {
  ival (nA)
  i (nA)
  amp (nA)
  noise (nA)
  on (1)
}

PROCEDURE set_noise_seed(x) {
  LOCAL lset_seed  
  lset_seed = x
  VERBATIM
  set_seed(_llset_seed);
  ENDVERBATIM
}

FUNCTION my_normrand(mean, std) {
    my_normrand = normrand(mean, std)
}

INITIAL {
  i = 0
  on = 0
  net_send(del, 1)
}

PROCEDURE seed(x) {
  set_seed(x)
}

BEFORE BREAKPOINT {
  if  (on) {
    noise = my_normrand(0,std*1(/nA))*1(nA)
    amp = f0 + 0.5*(f1-f0)*(tanh((t-torn)/(r/3)/(1(ms))-3)+1)
    ival = amp + noise + bias
  } else {
    ival = 0
  }
}

BREAKPOINT {
  i = ival
}

NET_RECEIVE (w) {
  if (flag == 1) {
    if (on == 0) {
      : turn it on
      on = 1
      : prepare to turn it off
      net_send(dur, 1)
    } else {
      : turn it off
      on = 0
    }
  }
}