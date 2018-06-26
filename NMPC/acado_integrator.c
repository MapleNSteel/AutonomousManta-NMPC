/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 27. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (tan(u[1]));
a[1] = (atan(((od[3]*a[0])/(od[2]+od[3]))));
a[2] = (sin(a[1]));
a[3] = (cos(xd[1]));
a[4] = (tan(u[1]));
a[5] = (atan(((od[3]*a[4])/(od[2]+od[3]))));
a[6] = (cos(a[5]));
a[7] = (sin(xd[1]));
a[8] = (tan(u[1]));
a[9] = (atan(((od[3]*a[8])/(od[2]+od[3]))));
a[10] = (sin(a[9]));
a[11] = (tan(u[1]));
a[12] = (atan(((od[3]*a[11])/(od[2]+od[3]))));
a[13] = (cos(a[12]));
a[14] = (cos(xd[1]));
a[15] = (tan(u[1]));
a[16] = (atan(((od[3]*a[15])/(od[2]+od[3]))));
a[17] = (sin(a[16]));
a[18] = (sin(xd[1]));
a[19] = (tan(u[1]));
a[20] = (atan(((od[3]*a[19])/(od[2]+od[3]))));
a[21] = (cos(a[20]));
a[22] = (cos(xd[1]));
a[23] = (tan(u[1]));
a[24] = (atan(((od[3]*a[23])/(od[2]+od[3]))));
a[25] = (sin(a[24]));
a[26] = (sin(xd[1]));

/* Compute outputs: */
out[0] = (((u[0]*a[2])*a[3])+((u[0]*a[6])*a[7]));
out[1] = (((u[0]*a[10])/od[3])-((((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18]))/(od[0]-xd[0])));
out[2] = ((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))/(od[0]-xd[0]));
out[3] = (real_t)(1.0000000000000000e+00);
}

void acado_rhs_ext(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 28;
const real_t* od = in + 30;
/* Vector of auxiliary variables; number of elements: 116. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (tan(u[1]));
a[1] = (atan(((od[3]*a[0])/(od[2]+od[3]))));
a[2] = (sin(a[1]));
a[3] = (cos(xd[1]));
a[4] = (tan(u[1]));
a[5] = (atan(((od[3]*a[4])/(od[2]+od[3]))));
a[6] = (cos(a[5]));
a[7] = (sin(xd[1]));
a[8] = (tan(u[1]));
a[9] = (atan(((od[3]*a[8])/(od[2]+od[3]))));
a[10] = (sin(a[9]));
a[11] = (tan(u[1]));
a[12] = (atan(((od[3]*a[11])/(od[2]+od[3]))));
a[13] = (cos(a[12]));
a[14] = (cos(xd[1]));
a[15] = (tan(u[1]));
a[16] = (atan(((od[3]*a[15])/(od[2]+od[3]))));
a[17] = (sin(a[16]));
a[18] = (sin(xd[1]));
a[19] = (tan(u[1]));
a[20] = (atan(((od[3]*a[19])/(od[2]+od[3]))));
a[21] = (cos(a[20]));
a[22] = (cos(xd[1]));
a[23] = (tan(u[1]));
a[24] = (atan(((od[3]*a[23])/(od[2]+od[3]))));
a[25] = (sin(a[24]));
a[26] = (sin(xd[1]));
a[27] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[28] = (xd[8]*a[27]);
a[29] = (cos(xd[1]));
a[30] = (xd[8]*a[29]);
a[31] = (xd[9]*a[27]);
a[32] = (xd[9]*a[29]);
a[33] = (xd[10]*a[27]);
a[34] = (xd[10]*a[29]);
a[35] = (xd[11]*a[27]);
a[36] = (xd[11]*a[29]);
a[37] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[38] = (xd[8]*a[37]);
a[39] = (cos(xd[1]));
a[40] = (xd[8]*a[39]);
a[41] = ((real_t)(1.0000000000000000e+00)/(od[0]-xd[0]));
a[42] = (a[41]*a[41]);
a[43] = (xd[9]*a[37]);
a[44] = (xd[9]*a[39]);
a[45] = (xd[10]*a[37]);
a[46] = (xd[10]*a[39]);
a[47] = (xd[11]*a[37]);
a[48] = (xd[11]*a[39]);
a[49] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[50] = (xd[8]*a[49]);
a[51] = (cos(xd[1]));
a[52] = (xd[8]*a[51]);
a[53] = ((real_t)(1.0000000000000000e+00)/(od[0]-xd[0]));
a[54] = (a[53]*a[53]);
a[55] = (xd[9]*a[49]);
a[56] = (xd[9]*a[51]);
a[57] = (xd[10]*a[49]);
a[58] = (xd[10]*a[51]);
a[59] = (xd[11]*a[49]);
a[60] = (xd[11]*a[51]);
a[61] = (xd[22]*a[27]);
a[62] = (xd[22]*a[29]);
a[63] = (xd[23]*a[27]);
a[64] = (xd[23]*a[29]);
a[65] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[66] = ((real_t)(1.0000000000000000e+00)/(od[2]+od[3]));
a[67] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[3]*a[0])/(od[2]+od[3])),2))));
a[68] = (((od[3]*a[65])*a[66])*a[67]);
a[69] = (cos(a[1]));
a[70] = (a[68]*a[69]);
a[71] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[72] = ((real_t)(1.0000000000000000e+00)/(od[2]+od[3]));
a[73] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[3]*a[4])/(od[2]+od[3])),2))));
a[74] = (((od[3]*a[71])*a[72])*a[73]);
a[75] = ((real_t)(-1.0000000000000000e+00)*(sin(a[5])));
a[76] = (a[74]*a[75]);
a[77] = (xd[22]*a[37]);
a[78] = (xd[22]*a[39]);
a[79] = ((real_t)(1.0000000000000000e+00)/od[3]);
a[80] = (xd[23]*a[37]);
a[81] = (xd[23]*a[39]);
a[82] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[83] = ((real_t)(1.0000000000000000e+00)/(od[2]+od[3]));
a[84] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[3]*a[8])/(od[2]+od[3])),2))));
a[85] = (((od[3]*a[82])*a[83])*a[84]);
a[86] = (cos(a[9]));
a[87] = (a[85]*a[86]);
a[88] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[89] = ((real_t)(1.0000000000000000e+00)/(od[2]+od[3]));
a[90] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[3]*a[11])/(od[2]+od[3])),2))));
a[91] = (((od[3]*a[88])*a[89])*a[90]);
a[92] = ((real_t)(-1.0000000000000000e+00)*(sin(a[12])));
a[93] = (a[91]*a[92]);
a[94] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[95] = ((real_t)(1.0000000000000000e+00)/(od[2]+od[3]));
a[96] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[3]*a[15])/(od[2]+od[3])),2))));
a[97] = (((od[3]*a[94])*a[95])*a[96]);
a[98] = (cos(a[16]));
a[99] = (a[97]*a[98]);
a[100] = (xd[22]*a[49]);
a[101] = (xd[22]*a[51]);
a[102] = (xd[23]*a[49]);
a[103] = (xd[23]*a[51]);
a[104] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[105] = ((real_t)(1.0000000000000000e+00)/(od[2]+od[3]));
a[106] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[3]*a[19])/(od[2]+od[3])),2))));
a[107] = (((od[3]*a[104])*a[105])*a[106]);
a[108] = ((real_t)(-1.0000000000000000e+00)*(sin(a[20])));
a[109] = (a[107]*a[108]);
a[110] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[111] = ((real_t)(1.0000000000000000e+00)/(od[2]+od[3]));
a[112] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[3]*a[23])/(od[2]+od[3])),2))));
a[113] = (((od[3]*a[110])*a[111])*a[112]);
a[114] = (cos(a[24]));
a[115] = (a[113]*a[114]);

/* Compute outputs: */
out[0] = (((u[0]*a[2])*a[3])+((u[0]*a[6])*a[7]));
out[1] = (((u[0]*a[10])/od[3])-((((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18]))/(od[0]-xd[0])));
out[2] = ((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))/(od[0]-xd[0]));
out[3] = (real_t)(1.0000000000000000e+00);
out[4] = (((u[0]*a[2])*a[28])+((u[0]*a[6])*a[30]));
out[5] = (((u[0]*a[2])*a[31])+((u[0]*a[6])*a[32]));
out[6] = (((u[0]*a[2])*a[33])+((u[0]*a[6])*a[34]));
out[7] = (((u[0]*a[2])*a[35])+((u[0]*a[6])*a[36]));
out[8] = ((real_t)(0.0000000000000000e+00)-(((((u[0]*a[13])*a[38])-((u[0]*a[17])*a[40]))*a[41])-(((((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18]))*((real_t)(0.0000000000000000e+00)-xd[4]))*a[42])));
out[9] = ((real_t)(0.0000000000000000e+00)-(((((u[0]*a[13])*a[43])-((u[0]*a[17])*a[44]))*a[41])-(((((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18]))*((real_t)(0.0000000000000000e+00)-xd[5]))*a[42])));
out[10] = ((real_t)(0.0000000000000000e+00)-(((((u[0]*a[13])*a[45])-((u[0]*a[17])*a[46]))*a[41])-(((((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18]))*((real_t)(0.0000000000000000e+00)-xd[6]))*a[42])));
out[11] = ((real_t)(0.0000000000000000e+00)-(((((u[0]*a[13])*a[47])-((u[0]*a[17])*a[48]))*a[41])-(((((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18]))*((real_t)(0.0000000000000000e+00)-xd[7]))*a[42])));
out[12] = (((od[0]*(((u[0]*a[21])*a[50])-((u[0]*a[25])*a[52])))*a[53])-(((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))*((real_t)(0.0000000000000000e+00)-xd[4]))*a[54]));
out[13] = (((od[0]*(((u[0]*a[21])*a[55])-((u[0]*a[25])*a[56])))*a[53])-(((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))*((real_t)(0.0000000000000000e+00)-xd[5]))*a[54]));
out[14] = (((od[0]*(((u[0]*a[21])*a[57])-((u[0]*a[25])*a[58])))*a[53])-(((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))*((real_t)(0.0000000000000000e+00)-xd[6]))*a[54]));
out[15] = (((od[0]*(((u[0]*a[21])*a[59])-((u[0]*a[25])*a[60])))*a[53])-(((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))*((real_t)(0.0000000000000000e+00)-xd[7]))*a[54]));
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = ((((u[0]*a[2])*a[61])+((u[0]*a[6])*a[62]))+((a[2]*a[3])+(a[6]*a[7])));
out[21] = ((((u[0]*a[2])*a[63])+((u[0]*a[6])*a[64]))+(((u[0]*a[70])*a[3])+((u[0]*a[76])*a[7])));
out[22] = (((real_t)(0.0000000000000000e+00)-(((((u[0]*a[13])*a[77])-((u[0]*a[17])*a[78]))*a[41])-(((((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18]))*((real_t)(0.0000000000000000e+00)-xd[20]))*a[42])))+((a[10]*a[79])-(((a[13]*a[14])-(a[17]*a[18]))*a[41])));
out[23] = (((real_t)(0.0000000000000000e+00)-(((((u[0]*a[13])*a[80])-((u[0]*a[17])*a[81]))*a[41])-(((((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18]))*((real_t)(0.0000000000000000e+00)-xd[21]))*a[42])))+(((u[0]*a[87])*a[79])-((((u[0]*a[93])*a[14])-((u[0]*a[99])*a[18]))*a[41])));
out[24] = ((((od[0]*(((u[0]*a[21])*a[100])-((u[0]*a[25])*a[101])))*a[53])-(((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))*((real_t)(0.0000000000000000e+00)-xd[20]))*a[54]))+((od[0]*((a[21]*a[22])-(a[25]*a[26])))*a[53]));
out[25] = ((((od[0]*(((u[0]*a[21])*a[102])-((u[0]*a[25])*a[103])))*a[53])-(((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))*((real_t)(0.0000000000000000e+00)-xd[21]))*a[54]))+((od[0]*(((u[0]*a[109])*a[22])-((u[0]*a[115])*a[26])))*a[53]));
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
}

/* Fixed step size:0.0333333 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int run1;
acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
rk_eta[4] = 1.0000000000000000e+00;
rk_eta[5] = 0.0000000000000000e+00;
rk_eta[6] = 0.0000000000000000e+00;
rk_eta[7] = 0.0000000000000000e+00;
rk_eta[8] = 0.0000000000000000e+00;
rk_eta[9] = 1.0000000000000000e+00;
rk_eta[10] = 0.0000000000000000e+00;
rk_eta[11] = 0.0000000000000000e+00;
rk_eta[12] = 0.0000000000000000e+00;
rk_eta[13] = 0.0000000000000000e+00;
rk_eta[14] = 1.0000000000000000e+00;
rk_eta[15] = 0.0000000000000000e+00;
rk_eta[16] = 0.0000000000000000e+00;
rk_eta[17] = 0.0000000000000000e+00;
rk_eta[18] = 0.0000000000000000e+00;
rk_eta[19] = 1.0000000000000000e+00;
rk_eta[20] = 0.0000000000000000e+00;
rk_eta[21] = 0.0000000000000000e+00;
rk_eta[22] = 0.0000000000000000e+00;
rk_eta[23] = 0.0000000000000000e+00;
rk_eta[24] = 0.0000000000000000e+00;
rk_eta[25] = 0.0000000000000000e+00;
rk_eta[26] = 0.0000000000000000e+00;
rk_eta[27] = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[28] = rk_eta[28];
acadoWorkspace.rk_xxx[29] = rk_eta[29];
acadoWorkspace.rk_xxx[30] = rk_eta[30];
acadoWorkspace.rk_xxx[31] = rk_eta[31];
acadoWorkspace.rk_xxx[32] = rk_eta[32];
acadoWorkspace.rk_xxx[33] = rk_eta[33];

for (run1 = 0; run1 < 1; ++run1)
{
acadoWorkspace.rk_xxx[0] = + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + rk_eta[20];
acadoWorkspace.rk_xxx[21] = + rk_eta[21];
acadoWorkspace.rk_xxx[22] = + rk_eta[22];
acadoWorkspace.rk_xxx[23] = + rk_eta[23];
acadoWorkspace.rk_xxx[24] = + rk_eta[24];
acadoWorkspace.rk_xxx[25] = + rk_eta[25];
acadoWorkspace.rk_xxx[26] = + rk_eta[26];
acadoWorkspace.rk_xxx[27] = + rk_eta[27];
acado_rhs_ext( acadoWorkspace.rk_xxx, acadoWorkspace.rk_kkk );
acadoWorkspace.rk_xxx[0] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[0] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[1] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[2] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[3] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[4] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[5] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[6] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[7] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[8] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[9] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[10] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[11] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[12] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[13] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[14] + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[15] + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[16] + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[17] + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[18] + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[19] + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[20] + rk_eta[20];
acadoWorkspace.rk_xxx[21] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[21] + rk_eta[21];
acadoWorkspace.rk_xxx[22] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[22] + rk_eta[22];
acadoWorkspace.rk_xxx[23] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[23] + rk_eta[23];
acadoWorkspace.rk_xxx[24] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[24] + rk_eta[24];
acadoWorkspace.rk_xxx[25] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[25] + rk_eta[25];
acadoWorkspace.rk_xxx[26] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[26] + rk_eta[26];
acadoWorkspace.rk_xxx[27] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[27] + rk_eta[27];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 28 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[28] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[29] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[30] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[31] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[32] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[33] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[34] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[35] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[36] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[37] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[38] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[39] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[40] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[41] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[42] + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[43] + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[44] + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[45] + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[46] + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[47] + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[48] + rk_eta[20];
acadoWorkspace.rk_xxx[21] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[49] + rk_eta[21];
acadoWorkspace.rk_xxx[22] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[50] + rk_eta[22];
acadoWorkspace.rk_xxx[23] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[51] + rk_eta[23];
acadoWorkspace.rk_xxx[24] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[52] + rk_eta[24];
acadoWorkspace.rk_xxx[25] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[53] + rk_eta[25];
acadoWorkspace.rk_xxx[26] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[54] + rk_eta[26];
acadoWorkspace.rk_xxx[27] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[55] + rk_eta[27];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 56 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[56] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[57] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[58] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[59] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[60] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[61] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[62] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[63] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[64] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[65] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[66] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[67] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[68] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[69] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[70] + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[71] + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[72] + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[73] + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[74] + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[75] + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[76] + rk_eta[20];
acadoWorkspace.rk_xxx[21] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[77] + rk_eta[21];
acadoWorkspace.rk_xxx[22] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[78] + rk_eta[22];
acadoWorkspace.rk_xxx[23] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[79] + rk_eta[23];
acadoWorkspace.rk_xxx[24] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[80] + rk_eta[24];
acadoWorkspace.rk_xxx[25] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[81] + rk_eta[25];
acadoWorkspace.rk_xxx[26] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[82] + rk_eta[26];
acadoWorkspace.rk_xxx[27] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[83] + rk_eta[27];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 84 ]) );
rk_eta[0] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[0] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[28] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[56] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[84];
rk_eta[1] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[1] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[29] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[57] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[85];
rk_eta[2] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[2] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[30] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[58] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[86];
rk_eta[3] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[3] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[31] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[59] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[87];
rk_eta[4] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[4] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[32] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[60] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[88];
rk_eta[5] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[5] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[33] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[61] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[89];
rk_eta[6] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[6] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[34] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[62] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[90];
rk_eta[7] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[7] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[35] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[63] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[91];
rk_eta[8] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[8] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[36] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[64] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[92];
rk_eta[9] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[9] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[37] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[65] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[93];
rk_eta[10] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[10] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[38] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[66] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[94];
rk_eta[11] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[11] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[39] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[67] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[95];
rk_eta[12] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[12] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[40] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[68] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[96];
rk_eta[13] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[13] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[41] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[69] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[97];
rk_eta[14] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[14] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[42] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[70] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[98];
rk_eta[15] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[15] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[43] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[71] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[99];
rk_eta[16] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[16] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[44] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[72] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[100];
rk_eta[17] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[17] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[45] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[73] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[101];
rk_eta[18] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[18] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[46] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[74] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[102];
rk_eta[19] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[19] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[47] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[75] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[103];
rk_eta[20] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[20] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[48] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[76] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[104];
rk_eta[21] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[21] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[49] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[77] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[105];
rk_eta[22] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[22] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[50] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[78] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[106];
rk_eta[23] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[23] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[51] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[79] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[107];
rk_eta[24] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[24] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[52] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[80] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[108];
rk_eta[25] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[25] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[53] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[81] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[109];
rk_eta[26] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[26] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[54] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[82] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[110];
rk_eta[27] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[27] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[55] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[83] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[111];
acadoWorkspace.rk_ttt += 1.0000000000000000e+00;
}
error = 0;
return error;
}

