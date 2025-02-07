/* Created by Language version: 7.7.0 */
/* NOT VECTORIZED */
#define NRN_VECTORIZED 0
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mech_api.h"
#undef PI
#define nil 0
#define _pval pval
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if !NRNGPU
#undef exp
#define exp hoc_Exp
#endif
 
#define nrn_init _nrn_init__NMDA_Channel_Calcium
#define _nrn_initial _nrn_initial__NMDA_Channel_Calcium
#define nrn_cur _nrn_cur__NMDA_Channel_Calcium
#define _nrn_current _nrn_current__NMDA_Channel_Calcium
#define nrn_jacob _nrn_jacob__NMDA_Channel_Calcium
#define nrn_state _nrn_state__NMDA_Channel_Calcium
#define _net_receive _net_receive__NMDA_Channel_Calcium 
#define state state__NMDA_Channel_Calcium 
 
#define _threadargscomma_ /**/
#define _threadargsprotocomma_ /**/
#define _threadargs_ /**/
#define _threadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *hoc_getarg(int);
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define gmax _p[0]
#define gmax_columnindex 0
#define e _p[1]
#define e_columnindex 1
#define depth _p[2]
#define depth_columnindex 2
#define taur _p[3]
#define taur_columnindex 3
#define mg _p[4]
#define mg_columnindex 4
#define alpha_mg _p[5]
#define alpha_mg_columnindex 5
#define beta_mg _p[6]
#define beta_mg_columnindex 6
#define i _p[7]
#define i_columnindex 7
#define mgblock _p[8]
#define mgblock_columnindex 8
#define local_cai _p[9]
#define local_cai_columnindex 9
#define g _p[10]
#define g_columnindex 10
#define drive_ca _p[11]
#define drive_ca_columnindex 11
#define Dlocal_cai _p[12]
#define Dlocal_cai_columnindex 12
#define _g _p[13]
#define _g_columnindex 13
#define _nd_area  *_ppvar[0].get<double*>()
#define receptor_activation	*_ppvar[2].get<double*>()
#define _p_receptor_activation _ppvar[2].literal_value<void*>()
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 static int hoc_nrnpointerindex =  2;
 /* external NEURON variables */
 /* declaration of user functions */
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mechtype);
#endif
 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(Object* _ho) { void* create_point_process(int, Object*);
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt(void*);
 static double _hoc_loc_pnt(void* _vptr) {double loc_point_process(int, void*);
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(void* _vptr) {double has_loc_point(void*);
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(void* _vptr) {
 double get_loc_point_process(void*); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _p = _prop->param; _ppvar = _prop->dparam;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 {0, 0}
};
 static Member_func _member_func[] = {
 {"loc", _hoc_loc_pnt},
 {"has_loc", _hoc_has_loc},
 {"get_loc", _hoc_get_loc_pnt},
 {0, 0}
};
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"gmax", "uS"},
 {"e", "mV"},
 {"depth", "um"},
 {"taur", "ms"},
 {"mg", "mM"},
 {"alpha_mg", "/mV"},
 {"local_cai", "mM"},
 {"i", "nA"},
 {"receptor_activation", "1"},
 {0, 0}
};
 static double delta_t = 0.01;
 static double local_cai0 = 0;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {0, 0}
};
 static DoubVec hoc_vdoub[] = {
 {0, 0, 0}
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(NrnThread*, Memb_list*, int);
static void nrn_state(NrnThread*, Memb_list*, int);
 static void nrn_cur(NrnThread*, Memb_list*, int);
static void  nrn_jacob(NrnThread*, Memb_list*, int);
 static void _hoc_destroy_pnt(void* _vptr) {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(NrnThread*, Memb_list*, int);
static void _ode_matsol(NrnThread*, Memb_list*, int);
 
#define _cvode_ieq _ppvar[3].literal_value<int>()
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"NMDA_Channel_Calcium",
 "gmax",
 "e",
 "depth",
 "taur",
 "mg",
 "alpha_mg",
 "beta_mg",
 0,
 "i",
 "mgblock",
 0,
 "local_cai",
 0,
 "receptor_activation",
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 14, _prop);
 	/*initialize range parameters*/
 	gmax = 0.1;
 	e = 0;
 	depth = 0.1;
 	taur = 200;
 	mg = 1;
 	alpha_mg = 0.062;
 	beta_mg = 3.57;
  }
 	_prop->param = _p;
 	_prop->param_size = 14;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 {0, 0}
};
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _NMDA_Channel_Calcium_reg() {
	int _vectorized = 0;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 0,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
  hoc_register_prop_size(_mechtype, 14, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "pointer");
  hoc_register_dparam_semantics(_mechtype, 3, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 NMDA_Channel_Calcium NMDA_Channel_Calcium.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 96485.3;
static int _reset;
static const char *modelname = "NMDA Receptor with Calcium Dynamics";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[1], _dlist1[1];
 static int state(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   Dlocal_cai = drive_ca - local_cai / taur ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 Dlocal_cai = Dlocal_cai  / (1. - dt*( ( - ( 1.0 ) / taur ) )) ;
  return 0;
}
 /*END CVODE*/
 static int state () {_reset=0;
 {
    local_cai = local_cai + (1. - exp(dt*(( - ( 1.0 ) / taur ))))*(- ( drive_ca ) / ( ( - ( 1.0 ) / taur ) ) - local_cai) ;
   }
  return 0;
}
 
static int _ode_count(int _type){ return 1;}
 
static void _ode_spec(NrnThread* _nt, Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 ();
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 1; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 ();
 }
 
static void _ode_matsol(NrnThread* _nt, Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  local_cai = local_cai0;
  _sav_indep = t; t = _save;

}
}

static void nrn_init(NrnThread* _nt, Memb_list* _ml, int _type){
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel();
}}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   mgblock = 1.0 / ( 1.0 + mg * beta_mg * exp ( - alpha_mg * v ) ) ;
   g = gmax * receptor_activation * mgblock ;
   i = g * ( v - e ) ;
   drive_ca = - g * ( v - 140.0 ) / ( 2.0 * FARADAY * depth ) ;
   if ( drive_ca <= 0.0 ) {
     drive_ca = 0.0 ;
     }
   }
 _current += i;

} return _current;
}

static void nrn_cur(NrnThread* _nt, Memb_list* _ml, int _type){
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_v + .001);
 	{ _rhs = _nrn_current(_v);
 	}
 _g = (_g - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}}

static void nrn_jacob(NrnThread* _nt, Memb_list* _ml, int _type){
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}}

static void nrn_state(NrnThread* _nt, Memb_list* _ml, int _type){
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 { error =  state();
 if(error){fprintf(stderr,"at line 37 in file NMDA_Channel_Calcium.mod:\nBREAKPOINT {\n"); nrn_complain(_p); abort_run(error);}
 }}}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = local_cai_columnindex;  _dlist1[0] = Dlocal_cai_columnindex;
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "NMDA_Channel_Calcium.mod";
    const char* nmodl_file_text = 
  "TITLE NMDA Receptor with Calcium Dynamics\n"
  "\n"
  "NEURON {\n"
  "    POINT_PROCESS NMDA_Channel_Calcium\n"
  "    NONSPECIFIC_CURRENT i\n"
  "    RANGE gmax, e, mgblock, mg, alpha_mg, beta_mg, depth, taur, local_cai\n"
  "    POINTER receptor_activation\n"
  "}\n"
  "\n"
  "CONSTANT {\n"
  "    FARADAY = 96485.3 (coulomb/mol) : Faraday's constant\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    gmax = 0.1 (uS)          : Maximum total conductance\n"
  "    e = 0 (mV)               : Reversal potential\n"
  "    depth = 0.1 (um)         : Shell depth for calcium accumulation\n"
  "    taur = 200 (ms)          : Calcium decay time constant\n"
  "    mg = 1 (mM)              : External [Mg2+]\n"
  "    alpha_mg = 0.062 (/mV)   : Voltage-dependence factor\n"
  "    beta_mg = 3.57           : Mg2+ block scaling\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    v (mV)                   : Membrane potential\n"
  "    i (nA)                   : Nonspecific current\n"
  "    g (uS)                   : Total channel conductance\n"
  "    mgblock                  : Magnesium block factor\n"
  "    drive_ca (mM/ms)         : Calcium influx rate\n"
  "    receptor_activation (1)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "    local_cai (mM)           : Localized calcium concentration\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "    SOLVE state METHOD cnexp\n"
  "\n"
  "    : Calculate Mg2+ block\n"
  "    mgblock = 1 / (1 + mg * beta_mg * exp(-alpha_mg * v))\n"
  "\n"
  "    : Total conductance\n"
  "    g = gmax * receptor_activation * mgblock\n"
  "\n"
  "    : Nonspecific current\n"
  "    i = g * (v - e)\n"
  "\n"
  "    : Calcium influx rate\n"
  "    drive_ca = -g * (v - 140) / (2 * FARADAY * depth)\n"
  "\n"
  "    : Prevent unphysical calcium extrusion\n"
  "    if (drive_ca <= 0) {\n"
  "        drive_ca = 0\n"
  "    }\n"
  "}\n"
  "\n"
  "DERIVATIVE state {\n"
  "    : Localized calcium dynamics\n"
  "    local_cai' = drive_ca - local_cai / taur\n"
  "}\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
