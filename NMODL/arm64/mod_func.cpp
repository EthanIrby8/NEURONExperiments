#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;
#if defined(__cplusplus)
extern "C" {
#endif

extern void _GABA_A_Channel_reg(void);
extern void _GenericLigand_reg(void);
extern void _NMDA16_2_reg(void);
extern void _NMDA_Channel_Calcium_reg(void);
extern void _SynNoise_reg(void);

void modl_reg() {
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");
    fprintf(stderr, " \"GABA_A_Channel.mod\"");
    fprintf(stderr, " \"GenericLigand.mod\"");
    fprintf(stderr, " \"NMDA16_2.mod\"");
    fprintf(stderr, " \"NMDA_Channel_Calcium.mod\"");
    fprintf(stderr, " \"SynNoise.mod\"");
    fprintf(stderr, "\n");
  }
  _GABA_A_Channel_reg();
  _GenericLigand_reg();
  _NMDA16_2_reg();
  _NMDA_Channel_Calcium_reg();
  _SynNoise_reg();
}

#if defined(__cplusplus)
}
#endif
