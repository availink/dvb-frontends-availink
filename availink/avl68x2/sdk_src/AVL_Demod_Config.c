
#include "AVL_Demod.h"

AVL_CommonConfig AVL_CommonConfigChip =
{
    0x14,
    Xtal_30M,
    
    AVL_TS_PARALLEL,
    AVL_MPCM_RISING,
    AVL_TS_CONTINUOUS_DISABLE
};

AVL_DVBTxConfig AVL_DVBTxConfigChip =
{
    AVL_IF_I,
    5*1000*1000,
    AVL_AGC_NORMAL//AVL_AGC_INVERTED
};

AVL_DVBSxConfig AVL_DVBSxConfigChip =
{
    AVL_AGC_INVERTED,
    AVL_DWM_Normal
};

AVL_ISDBTConfig AVL_ISDBTConfigChip =
{
    AVL_IF_I,
    AVL_ISDBT_BW_6M,
    5*1000*1000,
    AVL_AGC_NORMAL
};

AVL_DVBCConfig AVL_DVBCConfigChip =
{
    AVL_IF_I,
    5*1000*1000,
    6875*1000,
    AVL_AGC_NORMAL,
    AVL_DVBC_J83A
};

AVL_DTMBConfig AVL_DTMBConfigChip =
{
    AVL_IF_I,
    5*1000*1000,
    7560*1000,
    AVL_AGC_NORMAL
};



