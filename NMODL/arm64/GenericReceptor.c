/* Created by Language version: 7.7.0 */
/* NOT VECTORIZED */
#define NRN_VECTORIZED 0
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mech_api.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__GenericReceptor
#define _nrn_initial _nrn_initial__GenericReceptor
#define nrn_cur _nrn_cur__GenericReceptor
#define _nrn_current _nrn_current__GenericReceptor
#define nrn_jacob _nrn_jacob__GenericReceptor
#define nrn_state _nrn_state__GenericReceptor
#define _net_receive _net_receive__GenericReceptor 
#define states states__GenericReceptor 
 
#define _threadargscomma_ /**/
#define _threadargsprotocomma_ /**/
#define _threadargs_ /**/
#define _threadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define baseline_activity _p[0]
#define baseline_activity_columnindex 0
#define n_ligands _p[1]
#define n_ligands_columnindex 1
#define capacity _p[2]
#define capacity_columnindex 2
#define kd1 _p[3]
#define kd1_columnindex 3
#define efficacy1 _p[4]
#define efficacy1_columnindex 4
#define decay1 _p[5]
#define decay1_columnindex 5
#define kd2 _p[6]
#define kd2_columnindex 6
#define efficacy2 _p[7]
#define efficacy2_columnindex 7
#define decay2 _p[8]
#define decay2_columnindex 8
#define kd3 _p[9]
#define kd3_columnindex 9
#define efficacy3 _p[10]
#define efficacy3_columnindex 10
#define decay3 _p[11]
#define decay3_columnindex 11
#define kd4 _p[12]
#define kd4_columnindex 12
#define efficacy4 _p[13]
#define efficacy4_columnindex 13
#define decay4 _p[14]
#define decay4_columnindex 14
#define activation _p[15]
#define activation_columnindex 15
#define occupancy _p[16]
#define occupancy_columnindex 16
#define bound1 _p[17]
#define bound1_columnindex 17
#define bound2 _p[18]
#define bound2_columnindex 18
#define bound3 _p[19]
#define bound3_columnindex 19
#define bound4 _p[20]
#define bound4_columnindex 20
#define Dbound1 _p[21]
#define Dbound1_columnindex 21
#define Dbound2 _p[22]
#define Dbound2_columnindex 22
#define Dbound3 _p[23]
#define Dbound3_columnindex 23
#define Dbound4 _p[24]
#define Dbound4_columnindex 24
#define _g _p[25]
#define _g_columnindex 25
#define _nd_area  *_ppvar[0]._pval
#define C_lig1	*_ppvar[2]._pval
#define _p_C_lig1	_ppvar[2]._pval
#define C_lig2	*_ppvar[3]._pval
#define _p_C_lig2	_ppvar[3]._pval
#define C_lig3	*_ppvar[4]._pval
#define _p_C_lig3	_ppvar[4]._pval
#define C_lig4	*_ppvar[5]._pval
#define _p_C_lig4	_ppvar[5]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  2;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_max(void*);
 static double _hoc_min(void*);
 static double _hoc_occ(void*);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
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
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 "max", _hoc_max,
 "min", _hoc_min,
 "occ", _hoc_occ,
 0, 0
};
#define max max_GenericReceptor
#define min min_GenericReceptor
#define occ occ_GenericReceptor
 extern double max( double , double );
 extern double min( double , double );
 extern double occ( double , double );
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "kd1", "uM",
 "decay1", "/ms",
 "kd2", "uM",
 "decay2", "/ms",
 "kd3", "uM",
 "decay3", "/ms",
 "kd4", "uM",
 "decay4", "/ms",
 "bound1", "1",
 "bound2", "1",
 "bound3", "1",
 "bound4", "1",
 "activation", "1",
 "occupancy", "1",
 "C_lig1", "uM",
 "C_lig2", "uM",
 "C_lig3", "uM",
 "C_lig4", "uM",
 0,0
};
 static double bound40 = 0;
 static double bound30 = 0;
 static double bound20 = 0;
 static double bound10 = 0;
 static double delta_t = 0.01;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(NrnThread*, _Memb_list*, int);
static void nrn_state(NrnThread*, _Memb_list*, int);
 static void nrn_cur(NrnThread*, _Memb_list*, int);
static void  nrn_jacob(NrnThread*, _Memb_list*, int);
 static void _hoc_destroy_pnt(void* _vptr) {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(NrnThread*, _Memb_list*, int);
static void _ode_matsol(NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[6]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"GenericReceptor",
 "baseline_activity",
 "n_ligands",
 "capacity",
 "kd1",
 "efficacy1",
 "decay1",
 "kd2",
 "efficacy2",
 "decay2",
 "kd3",
 "efficacy3",
 "decay3",
 "kd4",
 "efficacy4",
 "decay4",
 0,
 "activation",
 "occupancy",
 0,
 "bound1",
 "bound2",
 "bound3",
 "bound4",
 0,
 "C_lig1",
 "C_lig2",
 "C_lig3",
 "C_lig4",
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
 	_p = nrn_prop_data_alloc(_mechtype, 26, _prop);
 	/*initialize range parameters*/
 	baseline_activity = 0;
 	n_ligands = 1;
 	capacity = 1;
 	kd1 = 0.5;
 	efficacy1 = 1;
 	decay1 = 0.1;
 	kd2 = 1;
 	efficacy2 = 0.5;
 	decay2 = 0.05;
 	kd3 = 0.7;
 	efficacy3 = -1;
 	decay3 = 0.2;
 	kd4 = 0.3;
 	efficacy4 = 0;
 	decay4 = 0.01;
  }
 	_prop->param = _p;
 	_prop->param_size = 26;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 7, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _GenericReceptor_reg() {
	int _vectorized = 0;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 0,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 26, 7);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "pointer");
  hoc_register_dparam_semantics(_mechtype, 3, "pointer");
  hoc_register_dparam_semantics(_mechtype, 4, "pointer");
  hoc_register_dparam_semantics(_mechtype, 5, "pointer");
  hoc_register_dparam_semantics(_mechtype, 6, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 GenericReceptor /Users/Ethan/Documents/Documents/GithubRepos/NEURONExperiments/NMODL/GenericReceptor.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "General Receptor with Per-Ligand Decay and Flexible Targeting";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[4], _dlist1[4];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   double _locc1 , _locc2 , _locc3 , _locc4 , _ltotal_demand , _lremaining_capacity , _lscale_factor ;
 if ( n_ligands >= 1.0 ) {
     _locc1 = occ ( _threadargscomma_ C_lig1 , kd1 ) ;
     }
   else {
     _locc1 = 0.0 ;
     }
   if ( n_ligands >= 2.0 ) {
     _locc2 = occ ( _threadargscomma_ C_lig2 , kd2 ) ;
     }
   else {
     _locc2 = 0.0 ;
     }
   if ( n_ligands >= 3.0 ) {
     _locc3 = occ ( _threadargscomma_ C_lig3 , kd3 ) ;
     }
   else {
     _locc3 = 0.0 ;
     }
   if ( n_ligands >= 4.0 ) {
     _locc4 = occ ( _threadargscomma_ C_lig4 , kd4 ) ;
     }
   else {
     _locc4 = 0.0 ;
     }
   _ltotal_demand = _locc1 + _locc2 + _locc3 + _locc4 ;
   _lremaining_capacity = capacity - occupancy ;
   if ( _ltotal_demand > _lremaining_capacity  && _ltotal_demand > 0.0 ) {
     _lscale_factor = _lremaining_capacity / _ltotal_demand ;
     }
   else {
     _lscale_factor = 1.0 ;
     }
   if ( n_ligands >= 1.0 ) {
     Dbound1 = _lscale_factor * _locc1 - bound1 * decay1 ;
     }
   if ( n_ligands >= 2.0 ) {
     Dbound2 = _lscale_factor * _locc2 - bound2 * decay2 ;
     }
   if ( n_ligands >= 3.0 ) {
     Dbound3 = _lscale_factor * _locc3 - bound3 * decay3 ;
     }
   if ( n_ligands >= 4.0 ) {
     Dbound4 = _lscale_factor * _locc4 - bound4 * decay4 ;
     }
   }
 return _reset;
}
 static int _ode_matsol1 () {
 double _locc1 , _locc2 , _locc3 , _locc4 , _ltotal_demand , _lremaining_capacity , _lscale_factor ;
 if ( n_ligands >= 1.0 ) {
   _locc1 = occ ( _threadargscomma_ C_lig1 , kd1 ) ;
   }
 else {
   _locc1 = 0.0 ;
   }
 if ( n_ligands >= 2.0 ) {
   _locc2 = occ ( _threadargscomma_ C_lig2 , kd2 ) ;
   }
 else {
   _locc2 = 0.0 ;
   }
 if ( n_ligands >= 3.0 ) {
   _locc3 = occ ( _threadargscomma_ C_lig3 , kd3 ) ;
   }
 else {
   _locc3 = 0.0 ;
   }
 if ( n_ligands >= 4.0 ) {
   _locc4 = occ ( _threadargscomma_ C_lig4 , kd4 ) ;
   }
 else {
   _locc4 = 0.0 ;
   }
 _ltotal_demand = _locc1 + _locc2 + _locc3 + _locc4 ;
 _lremaining_capacity = capacity - occupancy ;
 if ( _ltotal_demand > _lremaining_capacity  && _ltotal_demand > 0.0 ) {
   _lscale_factor = _lremaining_capacity / _ltotal_demand ;
   }
 else {
   _lscale_factor = 1.0 ;
   }
 if ( n_ligands >= 1.0 ) {
   Dbound1 = Dbound1  / (1. - dt*( ( - ( 1.0 )*( decay1 ) ) )) ;
   }
 if ( n_ligands >= 2.0 ) {
   Dbound2 = Dbound2  / (1. - dt*( ( - ( 1.0 )*( decay2 ) ) )) ;
   }
 if ( n_ligands >= 3.0 ) {
   Dbound3 = Dbound3  / (1. - dt*( ( - ( 1.0 )*( decay3 ) ) )) ;
   }
 if ( n_ligands >= 4.0 ) {
   Dbound4 = Dbound4  / (1. - dt*( ( - ( 1.0 )*( decay4 ) ) )) ;
   }
  return 0;
}
 /*END CVODE*/
 static int states () {_reset=0;
 {
   double _locc1 , _locc2 , _locc3 , _locc4 , _ltotal_demand , _lremaining_capacity , _lscale_factor ;
 if ( n_ligands >= 1.0 ) {
     _locc1 = occ ( _threadargscomma_ C_lig1 , kd1 ) ;
     }
   else {
     _locc1 = 0.0 ;
     }
   if ( n_ligands >= 2.0 ) {
     _locc2 = occ ( _threadargscomma_ C_lig2 , kd2 ) ;
     }
   else {
     _locc2 = 0.0 ;
     }
   if ( n_ligands >= 3.0 ) {
     _locc3 = occ ( _threadargscomma_ C_lig3 , kd3 ) ;
     }
   else {
     _locc3 = 0.0 ;
     }
   if ( n_ligands >= 4.0 ) {
     _locc4 = occ ( _threadargscomma_ C_lig4 , kd4 ) ;
     }
   else {
     _locc4 = 0.0 ;
     }
   _ltotal_demand = _locc1 + _locc2 + _locc3 + _locc4 ;
   _lremaining_capacity = capacity - occupancy ;
   if ( _ltotal_demand > _lremaining_capacity  && _ltotal_demand > 0.0 ) {
     _lscale_factor = _lremaining_capacity / _ltotal_demand ;
     }
   else {
     _lscale_factor = 1.0 ;
     }
   if ( n_ligands >= 1.0 ) {
      bound1 = bound1 + (1. - exp(dt*(( - ( 1.0 )*( decay1 ) ))))*(- ( ( _lscale_factor )*( _locc1 ) ) / ( ( - ( 1.0 )*( decay1 ) ) ) - bound1) ;
     }
   if ( n_ligands >= 2.0 ) {
      bound2 = bound2 + (1. - exp(dt*(( - ( 1.0 )*( decay2 ) ))))*(- ( ( _lscale_factor )*( _locc2 ) ) / ( ( - ( 1.0 )*( decay2 ) ) ) - bound2) ;
     }
   if ( n_ligands >= 3.0 ) {
      bound3 = bound3 + (1. - exp(dt*(( - ( 1.0 )*( decay3 ) ))))*(- ( ( _lscale_factor )*( _locc3 ) ) / ( ( - ( 1.0 )*( decay3 ) ) ) - bound3) ;
     }
   if ( n_ligands >= 4.0 ) {
      bound4 = bound4 + (1. - exp(dt*(( - ( 1.0 )*( decay4 ) ))))*(- ( ( _lscale_factor )*( _locc4 ) ) / ( ( - ( 1.0 )*( decay4 ) ) ) - bound4) ;
     }
   }
  return 0;
}
 
double occ (  double _lC_lig , double _lkd ) {
   double _locc;
 if ( _lC_lig + _lkd <= 0.0 ) {
     _locc = 0.0 ;
     }
   else {
     _locc = _lC_lig / ( _lkd + _lC_lig ) ;
     }
   
return _locc;
 }
 
static double _hoc_occ(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  occ (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
double min (  double _lx1 , double _lx2 ) {
   double _lmin;
 if ( _lx1 <= _lx2 ) {
     _lmin = _lx1 ;
     }
   else {
     _lmin = _lx2 ;
     }
   
return _lmin;
 }
 
static double _hoc_min(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  min (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
double max (  double _lx1 , double _lx2 ) {
   double _lmax;
 if ( _lx1 >= _lx2 ) {
     _lmax = _lx1 ;
     }
   else {
     _lmax = _lx2 ;
     }
   
return _lmax;
 }
 
static double _hoc_max(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  max (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
static int _ode_count(int _type){ return 4;}
 
static void _ode_spec(NrnThread* _nt, _Memb_list* _ml, int _type) {
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
	for (_i=0; _i < 4; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 ();
 }
 
static void _ode_matsol(NrnThread* _nt, _Memb_list* _ml, int _type) {
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
  bound4 = bound40;
  bound3 = bound30;
  bound2 = bound20;
  bound1 = bound10;
 {
   bound1 = 0.0 ;
   bound2 = 0.0 ;
   bound3 = 0.0 ;
   bound4 = 0.0 ;
   activation = 0.0 ;
   occupancy = 0.0 ;
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(NrnThread* _nt, _Memb_list* _ml, int _type){
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

static double _nrn_current(double _v){double _current=0.;v=_v;{
} return _current;
}

static void nrn_cur(NrnThread* _nt, _Memb_list* _ml, int _type){
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
 
}}

static void nrn_jacob(NrnThread* _nt, _Memb_list* _ml, int _type){
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

static void nrn_state(NrnThread* _nt, _Memb_list* _ml, int _type){
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
 { error =  states();
 if(error){fprintf(stderr,"at line 73 in file GenericReceptor.mod:\n    SOLVE states METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
 } {
   double _lnet_activation ;
 occupancy = bound1 + bound2 + bound3 + bound4 ;
   _lnet_activation = bound1 * efficacy1 + bound2 * efficacy2 + bound3 * efficacy3 + bound4 * efficacy4 ;
   activation = min ( _threadargscomma_ 1.0 , max ( _threadargscomma_ 0.0 , baseline_activity + _lnet_activation ) ) ;
   }
}}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = bound1_columnindex;  _dlist1[0] = Dbound1_columnindex;
 _slist1[1] = bound2_columnindex;  _dlist1[1] = Dbound2_columnindex;
 _slist1[2] = bound3_columnindex;  _dlist1[2] = Dbound3_columnindex;
 _slist1[3] = bound4_columnindex;  _dlist1[3] = Dbound4_columnindex;
_first = 0;
}

#if NMODL_TEXT
static const char* nmodl_filename = "/Users/Ethan/Documents/Documents/GithubRepos/NEURONExperiments/NMODL/GenericReceptor.mod";
static const char* nmodl_file_text = 
  "TITLE General Receptor with Per-Ligand Decay and Flexible Targeting\n"
  "\n"
  "NEURON {\n"
  "    POINT_PROCESS GenericReceptor\n"
  "    RANGE baseline_activity\n"
  "    RANGE activation, capacity, occupancy\n"
  "    RANGE n_ligands\n"
  "\n"
  "    : Parameters per ligand\n"
  "    RANGE kd1, efficacy1, bound1, decay1\n"
  "    RANGE kd2, efficacy2, bound2, decay2\n"
  "    RANGE kd3, efficacy3, bound3, decay3\n"
  "    RANGE kd4, efficacy4, bound4, decay4\n"
  "\n"
  "    POINTER C_lig1, C_lig2, C_lig3, C_lig4\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    baseline_activity = 0\n"
  "\n"
  "    n_ligands = 1         : Number of active ligands (1-4)\n"
  "\n"
  "    capacity = 1.0        : Max binding capacity (fractional, e.g., 1 = 100%)\n"
  "\n"
  "    : Ligand 1\n"
  "    kd1 = 0.5 (uM)\n"
  "    efficacy1 = 1.0\n"
  "    decay1 = 0.1 (/ms)\n"
  "\n"
  "    : Ligand 2\n"
  "    kd2 = 1.0 (uM)\n"
  "    efficacy2 = 0.5\n"
  "    decay2 = 0.05 (/ms)\n"
  "\n"
  "    : Ligand 3\n"
  "    kd3 = 0.7 (uM)\n"
  "    efficacy3 = -1.0\n"
  "    decay3 = 0.2 (/ms)\n"
  "\n"
  "    : Ligand 4\n"
  "    kd4 = 0.3 (uM)\n"
  "    efficacy4 = 0.0\n"
  "    decay4 = 0.01 (/ms)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    C_lig1 (uM)    : POINTER for Ligand 1\n"
  "    C_lig2 (uM)    : POINTER for Ligand 2\n"
  "    C_lig3 (uM)    : POINTER for Ligand 3\n"
  "    C_lig4 (uM)    : POINTER for Ligand 4\n"
  "    activation (1)  : Total activation level\n"
  "    occupancy (1)   : Total occupancy\n"
  "}\n"
  "\n"
  "STATE {\n"
  "    bound1 (1)      : Bound state for ligand 1\n"
  "    bound2 (1)      : Bound state for ligand 2\n"
  "    bound3 (1)      : Bound state for ligand 3\n"
  "    bound4 (1)      : Bound state for ligand 4\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "    bound1 = 0\n"
  "    bound2 = 0\n"
  "    bound3 = 0\n"
  "    bound4 = 0\n"
  "    activation = 0\n"
  "    occupancy = 0\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "    LOCAL net_activation\n"
  "    SOLVE states METHOD cnexp\n"
  "    occupancy = bound1 + bound2 + bound3 + bound4\n"
  "    net_activation = bound1 * efficacy1 + bound2 * efficacy2 + bound3 * efficacy3 + bound4 * efficacy4\n"
  "    activation = min(1, max(0, baseline_activity + net_activation))\n"
  "}\n"
  "\n"
  "DERIVATIVE states {\n"
  "    LOCAL occ1, occ2, occ3, occ4, total_demand, remaining_capacity, scale_factor\n"
  "\n"
  "    : Compute \"ideal\" contributions based on ligand affinity\n"
  "    if (n_ligands >= 1) {\n"
  "        occ1 = occ(C_lig1, kd1)\n"
  "    } else {\n"
  "        occ1 = 0\n"
  "    }\n"
  "    if (n_ligands >= 2) {\n"
  "        occ2 = occ(C_lig2, kd2)\n"
  "    } else {\n"
  "        occ2 = 0\n"
  "    }\n"
  "    if (n_ligands >= 3) {\n"
  "        occ3 = occ(C_lig3, kd3)\n"
  "    } else {\n"
  "        occ3 = 0\n"
  "    }\n"
  "    if (n_ligands >= 4) {\n"
  "        occ4 = occ(C_lig4, kd4)\n"
  "    } else {\n"
  "        occ4 = 0\n"
  "    }\n"
  "\n"
  "    : Sum total demand\n"
  "    total_demand = occ1 + occ2 + occ3 + occ4\n"
  "\n"
  "    remaining_capacity = capacity - occupancy\n"
  "\n"
  "    : Scale binding if demand exceeds capacity\n"
  "    if (total_demand > remaining_capacity && total_demand > 0) {\n"
  "        scale_factor = remaining_capacity / total_demand\n"
  "    }\n"
  "    else{\n"
  "        scale_factor = 1.0\n"
  "    }\n"
  "\n"
  "    : Apply scaled contributions\n"
  "    if (n_ligands >= 1) {\n"
  "        bound1' = scale_factor * occ1 - bound1 * decay1\n"
  "    }\n"
  "    if (n_ligands >= 2) {\n"
  "        bound2' = scale_factor * occ2 - bound2 * decay2\n"
  "    }\n"
  "    if (n_ligands >= 3) {\n"
  "        bound3' = scale_factor * occ3 - bound3 * decay3\n"
  "    }\n"
  "    if (n_ligands >= 4) {\n"
  "        bound4' = scale_factor * occ4 - bound4 * decay4\n"
  "    }\n"
  "}\n"
  "\n"
  "FUNCTION occ(C_lig(uM), kd(uM)) {\n"
  "    if (C_lig + kd <= 0) {\n"
  "        occ = 0\n"
  "    } else {\n"
  "        occ = C_lig / (kd + C_lig)\n"
  "    }\n"
  "}\n"
  "\n"
  "FUNCTION min(x1, x2) {\n"
  "    if (x1 <= x2){\n"
  "        min = x1\n"
  "    }\n"
  "    else{\n"
  "        min = x2\n"
  "    }\n"
  "}\n"
  "\n"
  "FUNCTION max(x1, x2) {\n"
  "    if (x1 >= x2){\n"
  "        max = x1\n"
  "    }\n"
  "    else{\n"
  "        max = x2\n"
  "    }\n"
  "}\n"
  ;
#endif
