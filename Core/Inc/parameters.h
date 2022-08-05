#ifndef PARAMETERS_H_
#define PARAMETERS_H_

// number of parameters
#define PARAMETER_ARRAY_SIZE         36

// definitions associating parameters with their names in the host file
typedef union {
    long a[PARAMETER_ARRAY_SIZE];
    struct {
    /*[0]*/ long Parameters;
    /*[1]*/ long InfoSwChecksum;
    /*[2]*/ float InfoSwVersion;
    /*[3]*/ long InfoCanNode;
    /*[4]*/ float PidPosPropGain;
    /*[5]*/ long PidPosPropLimit;
    /*[6]*/ long Reserved1;
    /*[7]*/ long Reserved2;
    /*[8]*/ long Reserved3;
    /*[9]*/ long Reserved4;
    /*[10]*/ long Reserved5;
    /*[11]*/ long Reserved6;
    /*[12]*/ long Reserved7;
    /*[13]*/ long Reserved8;
    /*[14]*/ long Reserved9;
    /*[15]*/ long Reserved10;
    /*[16]*/ long Reserved11;
    /*[17]*/ long Reserved12;
    /*[18]*/ long Reserved13;
    /*[19]*/ long Reserved14;
    /*[20]*/ long Reserved15;
    /*[21]*/ long Reserved16;
    /*[22]*/ long Reserved17;
    /*[23]*/ long Reserved18;
    /*[24]*/ long Reserved19;
    /*[25]*/ long Reserved20;
    /*[26]*/ long Reserved21;
    /*[27]*/ long Reserved22;
    /*[28]*/ long Reserved23;
    /*[29]*/ long Reserved24;
    /*[30]*/ long Reserved25;
    /*[31]*/ long Reserved26;
    /*[32]*/ long Reserved27;
    /*[33]*/ long Reserved28;
    /*[34]*/ long Reserved29;
    /*[35]*/ long Reserved30;
    } s;
} PARAMETER;

extern PARAMETER P;

#endif /* PARAMETERS_H_ */
