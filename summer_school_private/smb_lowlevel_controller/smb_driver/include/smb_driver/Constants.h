#ifndef __Constants__
#define __Constants__

namespace smb_driver {

/******************** Modes ********************/
//Command modes as defined by the Roboteq controller
//todo These need to be in a separate namespace!
enum HYAMBModes{
OPEN_LOOP = 0,
CLOSED_LOOP_SPEED,
CLOSED_LOOP_POSITION_RELATIVE,
CLOSED_LOOP_COUNT_POSITION,
CLOSED_LOOP_POSITION_TRACKING,
TORQUE,
CLOSED_LOOP_SPEED_POSITION
};

} // namespace smb_driver

/******************** SetCommand Device ********************/
#define _G  0

#define _M  1
#define _P  2
#define _S  3
#define _C  4
#define _CB  5
#define _VAR  6

#define _AC  7
#define _DC  8

#define _DS  9
#define _D1  10
#define _D0  11


#define _H  13
#define _EX  14
#define _MG  15
#define _MS  16

#define _PR  17
#define _PX  18
#define _PRX  19
#define _AX  20
#define _DX  21
#define _B  22
#define _SX  23

#define _CS  24


#define _RC  26

#define _EES  27
#define _BND  28


/******************** SetCommand Alias ********************/
#define _GO  0

#define _MOTCMD  1
#define _MOTPOS  2
#define _MOTVEL  3
#define _SENCNTR  4
#define _SBLCNTR  5
#define _VAR  6

#define _ACCEL  7
#define _DECEL  8

#define _DOUT  9
#define _DSET  10
#define _DRES  11


#define _HOME  13
#define _ESTOP  14
#define _MGO  15
#define _MSTOP  16

#define _MPOSREL  17
#define _NXTPOS  18
#define _NXTPOSR  19
#define _NXTACC  20
#define _NXTDEC  21
#define _BOOL  22
#define _NXTVEL  23

#define _CANSEND  24


#define _RCOUT  26

#define _EESAV  27
#define _BIND  28


/******************** GetValue Device ********************/
#define _A  0

#define _M  1
#define _P  2
#define _S  3
#define _C  4
#define _CB  5
#define _VAR  6

#define _SR  7
//#define _CR  8
#define _CBR  9
#define _BS  10
#define _BSR  11
#define _BA  12
#define _V  13
#define _D  14
#define _DI  15
#define _AI  16
#define _PI  17
#define _T  18
#define _F  19
#define _FS  20
#define _FF  21
#define _B  22
#define _DO  23
#define _E  24

#define _CIS  25
#define _CIA  26
#define _CIP  27

#define _TM  28

#define _LK  29




#define _TR  32

#define _K  33
#define _DR  34

#define _AIC  35
#define _PIC  36

#define _MA  37
#define _CL  38
#define _CAN  39
//#define _CF  40

#define _MGD  41
#define _MGT  42
#define _MGM  43
#define _MGS  44
#define _MGY  45




#define _FM  48
#define _HS  49


/******************** GetValue Alias ********************/
#define _MOTAMPS  0

#define _MOTCMD  1
#define _MOTPWR  2
#define _ABSPEED  3
#define _ABCNTR  4
#define _BLCNTR  5
#define _VAR  6

#define _RELSPEED  7
#define _RELCNTR  8
#define _BLRCNTR  9
#define _BLSPEED  10
#define _BLRSPEED  11
#define _BATAMPS  12
#define _VOLTS  13
#define _DIGIN  14
#define _DIN  15
#define _ANAIN  16
#define _PLSIN  17
#define _TEMP  18
#define _FEEDBK  19
#define _STFLAG  20
#define _FLTFLAG  21
#define _BOOL  22
#define _DIGOUT  23
#define _LPERR  24

#define _CMDSER  25
#define _CMDANA  26
#define _CMDPLS  27

#define _TIME  28

#define _LOCKED  29




#define _TRACK  32

#define _SPEKTRUM  33
#define _DREACHED  34

#define _ANAINC  35
#define _PLSINC  36

#define _MEMS  37
#define _CALIVE  38
#define _CAN  39
//#define _CF  40

#define _MGDET  41
#define _MGTRACK  42
#define _MGMRKR  43
#define _MGSTATUS  44
#define _MGYRO  45




#define _MOTFLAG  48
#define _HSENSE  49


/******************** SetConfig/GetConfig Alias ********************/
#define _EE  0

#define _BKD  1
#define _OVL  2
#define _UVL  3
#define _THLD  4
#define _MXMD  5
#define _PWMF  6

#define _CPRI  7
#define _RWD  8
#define _ECHOF  9
#define _RSBR  10
#define _ACS  11
#define _AMS  12
#define _CLIN  13
#define _DFC  14

#define _DINA  15
#define _DINL  16
#define _DOA  17
#define _DOL  18

#define _AMOD  19
#define _AMIN  20
#define _AMAX  21
#define _ACTR  22
#define _ADB  23
#define _ALIN  24
#define _AINA  25
#define _AMINA  26
#define _AMAXA  27
#define _APOL  28

#define _PMOD  29
#define _PMIN  30
#define _PMAX  31
#define _PCTR  32
#define _PDB  33
#define _PLIN  34
#define _PINA  35
#define _PMINA  36
#define _PMAXA  37
#define _PPOL  38

#define _MMOD  39
#define _MXPF  40
#define _MXPR  41
#define _ALIM  42
#define _ATRIG  43
#define _ATGA  44
#define _ATGD  45
#define _KP  46
#define _KI  47
#define _KD  48
#define _PIDM  49
#define _ICAP  50
#define _MAC  51
#define _MDEC  52
#define _MVEL  53
#define _MXRPM  54
#define _MXTRN  55
#define _CLERD  56

#define _BPOL  57
#define _BLSTD  58
#define _BLFB  59
#define _BHOME  60
#define _BLL  61
#define _BHL  62
#define _BLLA  63
#define _BHLA  64

#define _SXC  65
#define _SXM  66








#define _EMOD  72
#define _EPPR  73
#define _ELL  74
#define _EHL  75
#define _ELLA  76
#define _EHLA  77
#define _EHOME  78

#define _SKUSE  79
#define _SKMIN  80
#define _SKMAX  81
#define _SKCTR  82
#define _SKDB  83
#define _SKLIN  84

#define _CEN  85
#define _CNOD  86

#define _CHB  88
#define _CAS  89
#define _CLSN  90
#define _CSRT  91
#define _CTPS  92

#define _SCRO  93

#define _BMOD  94
#define _BADJ  95
#define _BADV  96
#define _BZPW  97


#endif
