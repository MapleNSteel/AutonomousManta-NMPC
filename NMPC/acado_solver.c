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




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
acadoWorkspace.state[0] = acadoVariables.x[0];
acadoWorkspace.state[1] = acadoVariables.x[1];
acadoWorkspace.state[2] = acadoVariables.x[2];
acadoWorkspace.state[3] = acadoVariables.x[3];
acadoWorkspace.state[28] = acadoVariables.u[0];
acadoWorkspace.state[29] = acadoVariables.u[1];
acadoWorkspace.state[30] = acadoVariables.od[0];
acadoWorkspace.state[31] = acadoVariables.od[1];
acadoWorkspace.state[32] = acadoVariables.od[2];
acadoWorkspace.state[33] = acadoVariables.od[3];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{

acadoWorkspace.state[28] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[29] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.state[30] = acadoVariables.od[lRun1 * 4];
acadoWorkspace.state[31] = acadoVariables.od[lRun1 * 4 + 1];
acadoWorkspace.state[32] = acadoVariables.od[lRun1 * 4 + 2];
acadoWorkspace.state[33] = acadoVariables.od[lRun1 * 4 + 3];

ret = acado_integrate(acadoWorkspace.state, lRun1 == 0);

acadoVariables.x[lRun1 * 4 + 4] = acadoWorkspace.state[0];
acadoVariables.x[lRun1 * 4 + 5] = acadoWorkspace.state[1];
acadoVariables.x[lRun1 * 4 + 6] = acadoWorkspace.state[2];
acadoVariables.x[lRun1 * 4 + 7] = acadoWorkspace.state[3];

acadoWorkspace.evGx[lRun1 * 16] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 16 + 1] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 16 + 2] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 16 + 3] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 16 + 4] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 16 + 5] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 16 + 6] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 16 + 7] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 16 + 8] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 16 + 9] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 16 + 10] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 16 + 11] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 16 + 12] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 16 + 13] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 16 + 14] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 16 + 15] = acadoWorkspace.state[19];

acadoWorkspace.evGu[lRun1 * 8] = acadoWorkspace.state[20];
acadoWorkspace.evGu[lRun1 * 8 + 1] = acadoWorkspace.state[21];
acadoWorkspace.evGu[lRun1 * 8 + 2] = acadoWorkspace.state[22];
acadoWorkspace.evGu[lRun1 * 8 + 3] = acadoWorkspace.state[23];
acadoWorkspace.evGu[lRun1 * 8 + 4] = acadoWorkspace.state[24];
acadoWorkspace.evGu[lRun1 * 8 + 5] = acadoWorkspace.state[25];
acadoWorkspace.evGu[lRun1 * 8 + 6] = acadoWorkspace.state[26];
acadoWorkspace.evGu[lRun1 * 8 + 7] = acadoWorkspace.state[27];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
const real_t* od = in + 6;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[3];
out[3] = (u[0]-od[1]);
out[4] = u[1];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[3];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 4];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 4 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 4 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 5] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 5 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 5 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 5 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 5 + 4] = acadoWorkspace.objValueOut[4];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acadoWorkspace.objValueIn[3] = acadoVariables.x[123];
acadoWorkspace.objValueIn[4] = acadoVariables.od[120];
acadoWorkspace.objValueIn[5] = acadoVariables.od[121];
acadoWorkspace.objValueIn[6] = acadoVariables.od[122];
acadoWorkspace.objValueIn[7] = acadoVariables.od[123];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3];
dNew[1] += + Gx1[4]*dOld[0] + Gx1[5]*dOld[1] + Gx1[6]*dOld[2] + Gx1[7]*dOld[3];
dNew[2] += + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3];
dNew[3] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[8] + Gx1[3]*Gx2[12];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[9] + Gx1[3]*Gx2[13];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[14];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[15];
Gx3[4] = + Gx1[4]*Gx2[0] + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[12];
Gx3[5] = + Gx1[4]*Gx2[1] + Gx1[5]*Gx2[5] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[13];
Gx3[6] = + Gx1[4]*Gx2[2] + Gx1[5]*Gx2[6] + Gx1[6]*Gx2[10] + Gx1[7]*Gx2[14];
Gx3[7] = + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[7] + Gx1[6]*Gx2[11] + Gx1[7]*Gx2[15];
Gx3[8] = + Gx1[8]*Gx2[0] + Gx1[9]*Gx2[4] + Gx1[10]*Gx2[8] + Gx1[11]*Gx2[12];
Gx3[9] = + Gx1[8]*Gx2[1] + Gx1[9]*Gx2[5] + Gx1[10]*Gx2[9] + Gx1[11]*Gx2[13];
Gx3[10] = + Gx1[8]*Gx2[2] + Gx1[9]*Gx2[6] + Gx1[10]*Gx2[10] + Gx1[11]*Gx2[14];
Gx3[11] = + Gx1[8]*Gx2[3] + Gx1[9]*Gx2[7] + Gx1[10]*Gx2[11] + Gx1[11]*Gx2[15];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[4] + Gx1[14]*Gx2[8] + Gx1[15]*Gx2[12];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[5] + Gx1[14]*Gx2[9] + Gx1[15]*Gx2[13];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[6] + Gx1[14]*Gx2[10] + Gx1[15]*Gx2[14];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[7] + Gx1[14]*Gx2[11] + Gx1[15]*Gx2[15];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7];
Gu2[2] = + Gx1[4]*Gu1[0] + Gx1[5]*Gu1[2] + Gx1[6]*Gu1[4] + Gx1[7]*Gu1[6];
Gu2[3] = + Gx1[4]*Gu1[1] + Gx1[5]*Gu1[3] + Gx1[6]*Gu1[5] + Gx1[7]*Gu1[7];
Gu2[4] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[6];
Gu2[5] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[7];
Gu2[6] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[2] + Gx1[14]*Gu1[4] + Gx1[15]*Gu1[6];
Gu2[7] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[5] + Gx1[15]*Gu1[7];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6];
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = (real_t)1.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = (real_t)1.0000000000000000e+00;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = acadoWorkspace.H[(iCol * 120) + (iRow * 2)];
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 120 + 60) + (iRow * 2)];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = acadoWorkspace.H[(iCol * 120) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 120 + 60) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = +dOld[0];
dNew[1] = +dOld[1];
dNew[2] = 0.0;
;
dNew[3] = +dOld[3];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = +Dy1[3];
RDy1[1] = +Dy1[4];
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = +Dy1[0];
QDy1[1] = +Dy1[1];
QDy1[2] = 0.0;
;
QDy1[3] = +Dy1[2];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[4] + E1[4]*Gx1[8] + E1[6]*Gx1[12];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[5] + E1[4]*Gx1[9] + E1[6]*Gx1[13];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[6] + E1[4]*Gx1[10] + E1[6]*Gx1[14];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[7] + E1[4]*Gx1[11] + E1[6]*Gx1[15];
H101[4] += + E1[1]*Gx1[0] + E1[3]*Gx1[4] + E1[5]*Gx1[8] + E1[7]*Gx1[12];
H101[5] += + E1[1]*Gx1[1] + E1[3]*Gx1[5] + E1[5]*Gx1[9] + E1[7]*Gx1[13];
H101[6] += + E1[1]*Gx1[2] + E1[3]*Gx1[6] + E1[5]*Gx1[10] + E1[7]*Gx1[14];
H101[7] += + E1[1]*Gx1[3] + E1[3]*Gx1[7] + E1[5]*Gx1[11] + E1[7]*Gx1[15];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 8; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
}

void acado_multQ1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = +Gx1[0];
Gx2[1] = +Gx1[1];
Gx2[2] = +Gx1[2];
Gx2[3] = +Gx1[3];
Gx2[4] = +Gx1[4];
Gx2[5] = +Gx1[5];
Gx2[6] = +Gx1[6];
Gx2[7] = +Gx1[7];
Gx2[8] = 0.0;
;
Gx2[9] = 0.0;
;
Gx2[10] = 0.0;
;
Gx2[11] = 0.0;
;
Gx2[12] = +Gx1[12];
Gx2[13] = +Gx1[13];
Gx2[14] = +Gx1[14];
Gx2[15] = +Gx1[15];
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)5.0000000000000000e+00*Gx1[0];
Gx2[1] = + (real_t)5.0000000000000000e+00*Gx1[1];
Gx2[2] = + (real_t)5.0000000000000000e+00*Gx1[2];
Gx2[3] = + (real_t)5.0000000000000000e+00*Gx1[3];
Gx2[4] = + (real_t)5.0000000000000000e+00*Gx1[4];
Gx2[5] = + (real_t)5.0000000000000000e+00*Gx1[5];
Gx2[6] = + (real_t)5.0000000000000000e+00*Gx1[6];
Gx2[7] = + (real_t)5.0000000000000000e+00*Gx1[7];
Gx2[8] = 0.0;
;
Gx2[9] = 0.0;
;
Gx2[10] = 0.0;
;
Gx2[11] = 0.0;
;
Gx2[12] = + (real_t)5.0000000000000000e+00*Gx1[12];
Gx2[13] = + (real_t)5.0000000000000000e+00*Gx1[13];
Gx2[14] = + (real_t)5.0000000000000000e+00*Gx1[14];
Gx2[15] = + (real_t)5.0000000000000000e+00*Gx1[15];
}

void acado_multQ1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = +Gu1[0];
Gu2[1] = +Gu1[1];
Gu2[2] = +Gu1[2];
Gu2[3] = +Gu1[3];
Gu2[4] = 0.0;
;
Gu2[5] = 0.0;
;
Gu2[6] = +Gu1[6];
Gu2[7] = +Gu1[7];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)5.0000000000000000e+00*Gu1[0];
Gu2[1] = + (real_t)5.0000000000000000e+00*Gu1[1];
Gu2[2] = + (real_t)5.0000000000000000e+00*Gu1[2];
Gu2[3] = + (real_t)5.0000000000000000e+00*Gu1[3];
Gu2[4] = 0.0;
;
Gu2[5] = 0.0;
;
Gu2[6] = + (real_t)5.0000000000000000e+00*Gu1[6];
Gu2[7] = + (real_t)5.0000000000000000e+00*Gu1[7];
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[4] + Hx[2]*Gx[8] + Hx[3]*Gx[12];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[5] + Hx[2]*Gx[9] + Hx[3]*Gx[13];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[6] + Hx[2]*Gx[10] + Hx[3]*Gx[14];
A01[3] = + Hx[0]*Gx[3] + Hx[1]*Gx[7] + Hx[2]*Gx[11] + Hx[3]*Gx[15];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 60 + 3600) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2] + Hx[2]*E[4] + Hx[3]*E[6];
acadoWorkspace.A[(row * 60 + 3600) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3] + Hx[2]*E[5] + Hx[3]*E[7];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 54. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (tan(u[1]));
a[1] = (atan(((od[3]*a[0])/(od[2]+od[3]))));
a[2] = (cos(a[1]));
a[3] = (cos(xd[1]));
a[4] = (tan(u[1]));
a[5] = (atan(((od[3]*a[4])/(od[2]+od[3]))));
a[6] = (sin(a[5]));
a[7] = (sin(xd[1]));
a[8] = (real_t)(-1.0000000000000000e+00);
a[9] = ((real_t)(1.0000000000000000e+00)/(od[0]-xd[0]));
a[10] = (a[9]*a[9]);
a[11] = ((real_t)(-1.0000000000000000e+00)*a[10]);
a[12] = (a[11]*(od[0]*(((u[0]*a[2])*a[3])-((u[0]*a[6])*a[7]))));
a[13] = (a[8]*a[12]);
a[14] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[15] = (u[0]*a[2]);
a[16] = (a[14]*a[15]);
a[17] = (cos(xd[1]));
a[18] = (u[0]*a[6]);
a[19] = (a[17]*a[18]);
a[20] = (real_t)(-1.0000000000000000e+00);
a[21] = (a[19]*a[20]);
a[22] = ((a[16]+a[21])*od[0]);
a[23] = (a[22]*a[9]);
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = (real_t)(0.0000000000000000e+00);
a[26] = (a[2]*a[3]);
a[27] = (a[6]*a[7]);
a[28] = (a[27]*a[20]);
a[29] = ((a[26]+a[28])*od[0]);
a[30] = (a[29]*a[9]);
a[31] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[32] = (a[31]*od[3]);
a[33] = ((real_t)(1.0000000000000000e+00)/(od[2]+od[3]));
a[34] = (a[32]*a[33]);
a[35] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[3]*a[0])/(od[2]+od[3])),2))));
a[36] = (a[34]*a[35]);
a[37] = ((real_t)(-1.0000000000000000e+00)*(sin(a[1])));
a[38] = (a[36]*a[37]);
a[39] = (a[38]*u[0]);
a[40] = (a[39]*a[3]);
a[41] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[42] = (a[41]*od[3]);
a[43] = ((real_t)(1.0000000000000000e+00)/(od[2]+od[3]));
a[44] = (a[42]*a[43]);
a[45] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[3]*a[4])/(od[2]+od[3])),2))));
a[46] = (a[44]*a[45]);
a[47] = (cos(a[5]));
a[48] = (a[46]*a[47]);
a[49] = (a[48]*u[0]);
a[50] = (a[49]*a[7]);
a[51] = (a[50]*a[20]);
a[52] = ((a[40]+a[51])*od[0]);
a[53] = (a[52]*a[9]);

/* Compute outputs: */
out[0] = ((od[0]*(((u[0]*a[2])*a[3])-((u[0]*a[6])*a[7])))/(od[0]-xd[0]));
out[1] = a[13];
out[2] = a[23];
out[3] = a[24];
out[4] = a[25];
out[5] = a[30];
out[6] = a[53];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 60 */
static const int xBoundIndices[ 60 ] = 
{ 4, 5, 8, 9, 12, 13, 16, 17, 20, 21, 24, 25, 28, 29, 32, 33, 36, 37, 40, 41, 44, 45, 48, 49, 52, 53, 56, 57, 60, 61, 64, 65, 68, 69, 72, 73, 76, 77, 80, 81, 84, 85, 88, 89, 92, 93, 96, 97, 100, 101, 104, 105, 108, 109, 112, 113, 116, 117, 120, 121 };
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
for (lRun1 = 1; lRun1 < 30; ++lRun1)
{
acado_moveGxT( &(acadoWorkspace.evGx[ lRun1 * 16 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ lRun1 * 16-16 ]), &(acadoWorkspace.evGx[ lRun1 * 16 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ lRun4 * 8 ]), &(acadoWorkspace.E[ lRun3 * 8 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun1 * 8 ]), &(acadoWorkspace.E[ lRun3 * 8 ]) );
}

for (lRun1 = 0; lRun1 < 29; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multQ1Gu( &(acadoWorkspace.E[ lRun3 * 8 ]), &(acadoWorkspace.QE[ lRun3 * 8 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multQN1Gu( &(acadoWorkspace.E[ lRun3 * 8 ]), &(acadoWorkspace.QE[ lRun3 * 8 ]) );
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acado_zeroBlockH10( &(acadoWorkspace.H10[ lRun1 * 8 ]) );
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multQETGx( &(acadoWorkspace.QE[ lRun3 * 8 ]), &(acadoWorkspace.evGx[ lRun2 * 16 ]), &(acadoWorkspace.H10[ lRun1 * 8 ]) );
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acado_setBlockH11_R1( lRun1, lRun1 );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 8 ]), &(acadoWorkspace.QE[ lRun5 * 8 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 30; ++lRun2)
{
acado_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 8 ]), &(acadoWorkspace.QE[ lRun5 * 8 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun1, lRun2 );
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_macETSlu( &(acadoWorkspace.QE[ lRun3 * 8 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}
acadoWorkspace.lb[0] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[43];
acadoWorkspace.lb[44] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[46];
acadoWorkspace.lb[47] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[47];
acadoWorkspace.lb[48] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[48];
acadoWorkspace.lb[49] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[49];
acadoWorkspace.lb[50] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[50];
acadoWorkspace.lb[51] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[51];
acadoWorkspace.lb[52] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[52];
acadoWorkspace.lb[53] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[53];
acadoWorkspace.lb[54] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[54];
acadoWorkspace.lb[55] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[55];
acadoWorkspace.lb[56] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[56];
acadoWorkspace.lb[57] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[57];
acadoWorkspace.lb[58] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[58];
acadoWorkspace.lb[59] = (real_t)-5.2000000000000002e-01 - acadoVariables.u[59];
acadoWorkspace.ub[0] = (real_t)1.0000000000000000e+12 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)5.2000000000000002e-01 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)5.2000000000000002e-01 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)1.0000000000000000e+12 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)5.2000000000000002e-01 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+12 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)5.2000000000000002e-01 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)5.2000000000000002e-01 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+12 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)5.2000000000000002e-01 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)1.0000000000000000e+12 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)5.2000000000000002e-01 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)5.2000000000000002e-01 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)1.0000000000000000e+12 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)5.2000000000000002e-01 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+12 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)5.2000000000000002e-01 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)1.0000000000000000e+12 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)5.2000000000000002e-01 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)1.0000000000000000e+12 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)5.2000000000000002e-01 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)1.0000000000000000e+12 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)5.2000000000000002e-01 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e+12 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)5.2000000000000002e-01 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)1.0000000000000000e+12 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)5.2000000000000002e-01 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)1.0000000000000000e+12 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)5.2000000000000002e-01 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)1.0000000000000000e+12 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)5.2000000000000002e-01 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)1.0000000000000000e+12 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)5.2000000000000002e-01 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)1.0000000000000000e+12 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)5.2000000000000002e-01 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)1.0000000000000000e+12 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)5.2000000000000002e-01 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)1.0000000000000000e+12 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)5.2000000000000002e-01 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)1.0000000000000000e+12 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)5.2000000000000002e-01 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)1.0000000000000000e+12 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)5.2000000000000002e-01 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)1.0000000000000000e+12 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)5.2000000000000002e-01 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)1.0000000000000000e+12 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)5.2000000000000002e-01 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)1.0000000000000000e+12 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)5.2000000000000002e-01 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)1.0000000000000000e+12 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)5.2000000000000002e-01 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)1.0000000000000000e+12 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)5.2000000000000002e-01 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)1.0000000000000000e+12 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)5.2000000000000002e-01 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)1.0000000000000000e+12 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)5.2000000000000002e-01 - acadoVariables.u[59];

for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 4;
lRun4 = ((lRun3) / (4)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = (((((lRun4) * (lRun4-1)) / (2)) + (lRun2)) * (4)) + ((lRun3) % (4));
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 2)] = acadoWorkspace.E[lRun5 * 2];
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 2 + 1)] = acadoWorkspace.E[lRun5 * 2 + 1];
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.conValueIn[5] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.conValueIn[6] = acadoVariables.od[lRun1 * 4];
acadoWorkspace.conValueIn[7] = acadoVariables.od[lRun1 * 4 + 1];
acadoWorkspace.conValueIn[8] = acadoVariables.od[lRun1 * 4 + 2];
acadoWorkspace.conValueIn[9] = acadoVariables.od[lRun1 * 4 + 3];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1] = acadoWorkspace.conValueOut[0];

acadoWorkspace.evHx[lRun1 * 4] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evHx[lRun1 * 4 + 1] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun1 * 4 + 2] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 4 + 3] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHu[lRun1 * 2] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHu[lRun1 * 2 + 1] = acadoWorkspace.conValueOut[6];
}

acadoWorkspace.A01[0] = acadoWorkspace.evHx[0];
acadoWorkspace.A01[1] = acadoWorkspace.evHx[1];
acadoWorkspace.A01[2] = acadoWorkspace.evHx[2];
acadoWorkspace.A01[3] = acadoWorkspace.evHx[3];

acado_multHxC( &(acadoWorkspace.evHx[ 4 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 4 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 8 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.A01[ 8 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 12 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.A01[ 12 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 16 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.A01[ 16 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 20 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.A01[ 20 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.A01[ 24 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 28 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.A01[ 28 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 32 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.A01[ 32 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.A01[ 36 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.A01[ 40 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 44 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.A01[ 44 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.A01[ 48 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 52 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.A01[ 52 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.A01[ 56 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.A01[ 60 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.A01[ 64 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 68 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.A01[ 68 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.A01[ 72 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 76 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.A01[ 76 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.A01[ 80 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.A01[ 84 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 88 ]), &(acadoWorkspace.evGx[ 336 ]), &(acadoWorkspace.A01[ 88 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 92 ]), &(acadoWorkspace.evGx[ 352 ]), &(acadoWorkspace.A01[ 92 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.evGx[ 368 ]), &(acadoWorkspace.A01[ 96 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.A01[ 100 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 104 ]), &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.A01[ 104 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.evGx[ 416 ]), &(acadoWorkspace.A01[ 108 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 112 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.A01[ 112 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 116 ]), &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.A01[ 116 ]) );

for (lRun2 = 0; lRun2 < 29; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun3);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 * 4 + 4 ]), &(acadoWorkspace.E[ lRun4 * 8 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[3600] = acadoWorkspace.evHu[0];
acadoWorkspace.A[3601] = acadoWorkspace.evHu[1];
acadoWorkspace.A[3662] = acadoWorkspace.evHu[2];
acadoWorkspace.A[3663] = acadoWorkspace.evHu[3];
acadoWorkspace.A[3724] = acadoWorkspace.evHu[4];
acadoWorkspace.A[3725] = acadoWorkspace.evHu[5];
acadoWorkspace.A[3786] = acadoWorkspace.evHu[6];
acadoWorkspace.A[3787] = acadoWorkspace.evHu[7];
acadoWorkspace.A[3848] = acadoWorkspace.evHu[8];
acadoWorkspace.A[3849] = acadoWorkspace.evHu[9];
acadoWorkspace.A[3910] = acadoWorkspace.evHu[10];
acadoWorkspace.A[3911] = acadoWorkspace.evHu[11];
acadoWorkspace.A[3972] = acadoWorkspace.evHu[12];
acadoWorkspace.A[3973] = acadoWorkspace.evHu[13];
acadoWorkspace.A[4034] = acadoWorkspace.evHu[14];
acadoWorkspace.A[4035] = acadoWorkspace.evHu[15];
acadoWorkspace.A[4096] = acadoWorkspace.evHu[16];
acadoWorkspace.A[4097] = acadoWorkspace.evHu[17];
acadoWorkspace.A[4158] = acadoWorkspace.evHu[18];
acadoWorkspace.A[4159] = acadoWorkspace.evHu[19];
acadoWorkspace.A[4220] = acadoWorkspace.evHu[20];
acadoWorkspace.A[4221] = acadoWorkspace.evHu[21];
acadoWorkspace.A[4282] = acadoWorkspace.evHu[22];
acadoWorkspace.A[4283] = acadoWorkspace.evHu[23];
acadoWorkspace.A[4344] = acadoWorkspace.evHu[24];
acadoWorkspace.A[4345] = acadoWorkspace.evHu[25];
acadoWorkspace.A[4406] = acadoWorkspace.evHu[26];
acadoWorkspace.A[4407] = acadoWorkspace.evHu[27];
acadoWorkspace.A[4468] = acadoWorkspace.evHu[28];
acadoWorkspace.A[4469] = acadoWorkspace.evHu[29];
acadoWorkspace.A[4530] = acadoWorkspace.evHu[30];
acadoWorkspace.A[4531] = acadoWorkspace.evHu[31];
acadoWorkspace.A[4592] = acadoWorkspace.evHu[32];
acadoWorkspace.A[4593] = acadoWorkspace.evHu[33];
acadoWorkspace.A[4654] = acadoWorkspace.evHu[34];
acadoWorkspace.A[4655] = acadoWorkspace.evHu[35];
acadoWorkspace.A[4716] = acadoWorkspace.evHu[36];
acadoWorkspace.A[4717] = acadoWorkspace.evHu[37];
acadoWorkspace.A[4778] = acadoWorkspace.evHu[38];
acadoWorkspace.A[4779] = acadoWorkspace.evHu[39];
acadoWorkspace.A[4840] = acadoWorkspace.evHu[40];
acadoWorkspace.A[4841] = acadoWorkspace.evHu[41];
acadoWorkspace.A[4902] = acadoWorkspace.evHu[42];
acadoWorkspace.A[4903] = acadoWorkspace.evHu[43];
acadoWorkspace.A[4964] = acadoWorkspace.evHu[44];
acadoWorkspace.A[4965] = acadoWorkspace.evHu[45];
acadoWorkspace.A[5026] = acadoWorkspace.evHu[46];
acadoWorkspace.A[5027] = acadoWorkspace.evHu[47];
acadoWorkspace.A[5088] = acadoWorkspace.evHu[48];
acadoWorkspace.A[5089] = acadoWorkspace.evHu[49];
acadoWorkspace.A[5150] = acadoWorkspace.evHu[50];
acadoWorkspace.A[5151] = acadoWorkspace.evHu[51];
acadoWorkspace.A[5212] = acadoWorkspace.evHu[52];
acadoWorkspace.A[5213] = acadoWorkspace.evHu[53];
acadoWorkspace.A[5274] = acadoWorkspace.evHu[54];
acadoWorkspace.A[5275] = acadoWorkspace.evHu[55];
acadoWorkspace.A[5336] = acadoWorkspace.evHu[56];
acadoWorkspace.A[5337] = acadoWorkspace.evHu[57];
acadoWorkspace.A[5398] = acadoWorkspace.evHu[58];
acadoWorkspace.A[5399] = acadoWorkspace.evHu[59];
acadoWorkspace.lbA[60] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[0];
acadoWorkspace.lbA[61] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[1];
acadoWorkspace.lbA[62] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[2];
acadoWorkspace.lbA[63] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[3];
acadoWorkspace.lbA[64] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[4];
acadoWorkspace.lbA[65] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[5];
acadoWorkspace.lbA[66] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[6];
acadoWorkspace.lbA[67] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[7];
acadoWorkspace.lbA[68] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[8];
acadoWorkspace.lbA[69] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[9];
acadoWorkspace.lbA[70] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[10];
acadoWorkspace.lbA[71] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[11];
acadoWorkspace.lbA[72] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[12];
acadoWorkspace.lbA[73] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[13];
acadoWorkspace.lbA[74] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[14];
acadoWorkspace.lbA[75] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[15];
acadoWorkspace.lbA[76] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[16];
acadoWorkspace.lbA[77] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[17];
acadoWorkspace.lbA[78] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[18];
acadoWorkspace.lbA[79] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[19];
acadoWorkspace.lbA[80] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[20];
acadoWorkspace.lbA[81] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[21];
acadoWorkspace.lbA[82] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[22];
acadoWorkspace.lbA[83] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[23];
acadoWorkspace.lbA[84] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[24];
acadoWorkspace.lbA[85] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[25];
acadoWorkspace.lbA[86] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[26];
acadoWorkspace.lbA[87] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[27];
acadoWorkspace.lbA[88] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[28];
acadoWorkspace.lbA[89] = (real_t)1.0000000000000000e-03 - acadoWorkspace.evH[29];

acadoWorkspace.ubA[60] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[61] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[62] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[63] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[64] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[4];
acadoWorkspace.ubA[65] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[66] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[6];
acadoWorkspace.ubA[67] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[68] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[8];
acadoWorkspace.ubA[69] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[70] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[10];
acadoWorkspace.ubA[71] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[72] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[12];
acadoWorkspace.ubA[73] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[74] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[14];
acadoWorkspace.ubA[75] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[76] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[16];
acadoWorkspace.ubA[77] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[78] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[18];
acadoWorkspace.ubA[79] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[19];
acadoWorkspace.ubA[80] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[20];
acadoWorkspace.ubA[81] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[21];
acadoWorkspace.ubA[82] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[22];
acadoWorkspace.ubA[83] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[23];
acadoWorkspace.ubA[84] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[24];
acadoWorkspace.ubA[85] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[25];
acadoWorkspace.ubA[86] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[26];
acadoWorkspace.ubA[87] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[27];
acadoWorkspace.ubA[88] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[28];
acadoWorkspace.ubA[89] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[29];

}

void acado_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];

for (lRun2 = 0; lRun2 < 150; ++lRun2)
acadoWorkspace.Dy[lRun2] -= acadoVariables.y[lRun2];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 115 ]), &(acadoWorkspace.g[ 46 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 125 ]), &(acadoWorkspace.g[ 50 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 145 ]), &(acadoWorkspace.g[ 58 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.QDy[ 44 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.QDy[ 52 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 56 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 64 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.QDy[ 68 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.QDy[ 76 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.QDy[ 88 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 115 ]), &(acadoWorkspace.QDy[ 92 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 125 ]), &(acadoWorkspace.QDy[ 100 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.QDy[ 104 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 112 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 145 ]), &(acadoWorkspace.QDy[ 116 ]) );

acadoWorkspace.QDy[120] = + (real_t)5.0000000000000000e+00*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[121] = + (real_t)5.0000000000000000e+00*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[122] = 0.0;
;
acadoWorkspace.QDy[123] = + (real_t)5.0000000000000000e+00*acadoWorkspace.DyN[2];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multEQDy( &(acadoWorkspace.E[ lRun3 * 8 ]), &(acadoWorkspace.QDy[ lRun2 * 4 + 4 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[1] += + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[2] += + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[3] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[4] += + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[5] += + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[6] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[7] += + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[8] += + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[9] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[10] += + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[11] += + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[12] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[13] += + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[14] += + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[15] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[16] += + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[17] += + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[18] += + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[19] += + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[20] += + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[21] += + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[22] += + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[23] += + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[24] += + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[25] += + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[26] += + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[27] += + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[28] += + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[29] += + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[30] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[31] += + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[32] += + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[33] += + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[34] += + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[35] += + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[36] += + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[37] += + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[38] += + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[39] += + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[40] += + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[41] += + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[42] += + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[43] += + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[44] += + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[45] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[46] += + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[47] += + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[48] += + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[49] += + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[50] += + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[51] += + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[52] += + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[53] += + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[54] += + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[55] += + acadoWorkspace.H10[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[223]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[56] += + acadoWorkspace.H10[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[227]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[57] += + acadoWorkspace.H10[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[231]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[58] += + acadoWorkspace.H10[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[235]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[59] += + acadoWorkspace.H10[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[239]*acadoWorkspace.Dx0[3];

tmp = + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoVariables.x[4];
acadoWorkspace.lbA[0] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[0] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[3] + acadoVariables.x[5];
acadoWorkspace.lbA[1] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[1] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[3] + acadoVariables.x[8];
acadoWorkspace.lbA[2] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[2] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoVariables.x[9];
acadoWorkspace.lbA[3] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[3] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[3] + acadoVariables.x[12];
acadoWorkspace.lbA[4] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[4] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[3] + acadoVariables.x[13];
acadoWorkspace.lbA[5] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[5] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[3] + acadoVariables.x[16];
acadoWorkspace.lbA[6] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[6] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[3] + acadoVariables.x[17];
acadoWorkspace.lbA[7] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[7] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[3] + acadoVariables.x[20];
acadoWorkspace.lbA[8] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[8] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[3] + acadoVariables.x[21];
acadoWorkspace.lbA[9] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[9] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[3] + acadoVariables.x[24];
acadoWorkspace.lbA[10] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[10] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[3] + acadoVariables.x[25];
acadoWorkspace.lbA[11] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[11] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[3] + acadoVariables.x[28];
acadoWorkspace.lbA[12] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[12] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[3] + acadoVariables.x[29];
acadoWorkspace.lbA[13] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[13] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[3] + acadoVariables.x[32];
acadoWorkspace.lbA[14] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[14] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[3] + acadoVariables.x[33];
acadoWorkspace.lbA[15] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[15] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[3] + acadoVariables.x[36];
acadoWorkspace.lbA[16] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[16] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[3] + acadoVariables.x[37];
acadoWorkspace.lbA[17] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[17] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[3] + acadoVariables.x[40];
acadoWorkspace.lbA[18] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[18] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[3] + acadoVariables.x[41];
acadoWorkspace.lbA[19] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[19] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[3] + acadoVariables.x[44];
acadoWorkspace.lbA[20] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[20] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[3] + acadoVariables.x[45];
acadoWorkspace.lbA[21] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[21] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[3] + acadoVariables.x[48];
acadoWorkspace.lbA[22] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[22] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoVariables.x[49];
acadoWorkspace.lbA[23] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[23] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[3] + acadoVariables.x[52];
acadoWorkspace.lbA[24] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[24] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[3] + acadoVariables.x[53];
acadoWorkspace.lbA[25] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[25] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[3] + acadoVariables.x[56];
acadoWorkspace.lbA[26] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[26] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[3] + acadoVariables.x[57];
acadoWorkspace.lbA[27] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[27] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[3] + acadoVariables.x[60];
acadoWorkspace.lbA[28] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[28] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[3] + acadoVariables.x[61];
acadoWorkspace.lbA[29] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[29] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoVariables.x[64];
acadoWorkspace.lbA[30] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[30] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[3] + acadoVariables.x[65];
acadoWorkspace.lbA[31] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[31] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[3] + acadoVariables.x[68];
acadoWorkspace.lbA[32] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[32] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[3] + acadoVariables.x[69];
acadoWorkspace.lbA[33] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[33] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[3] + acadoVariables.x[72];
acadoWorkspace.lbA[34] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[34] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[3] + acadoVariables.x[73];
acadoWorkspace.lbA[35] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[35] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[3] + acadoVariables.x[76];
acadoWorkspace.lbA[36] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[36] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[3] + acadoVariables.x[77];
acadoWorkspace.lbA[37] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[37] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[3] + acadoVariables.x[80];
acadoWorkspace.lbA[38] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[38] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[3] + acadoVariables.x[81];
acadoWorkspace.lbA[39] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[39] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[3] + acadoVariables.x[84];
acadoWorkspace.lbA[40] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[40] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[3] + acadoVariables.x[85];
acadoWorkspace.lbA[41] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[41] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[3] + acadoVariables.x[88];
acadoWorkspace.lbA[42] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[42] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[3] + acadoVariables.x[89];
acadoWorkspace.lbA[43] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[43] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[3] + acadoVariables.x[92];
acadoWorkspace.lbA[44] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[44] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[3] + acadoVariables.x[93];
acadoWorkspace.lbA[45] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[45] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[3] + acadoVariables.x[96];
acadoWorkspace.lbA[46] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[46] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[3] + acadoVariables.x[97];
acadoWorkspace.lbA[47] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[47] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[3] + acadoVariables.x[100];
acadoWorkspace.lbA[48] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[48] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[3] + acadoVariables.x[101];
acadoWorkspace.lbA[49] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[49] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[403]*acadoWorkspace.Dx0[3] + acadoVariables.x[104];
acadoWorkspace.lbA[50] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[50] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[404]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[405]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[406]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[407]*acadoWorkspace.Dx0[3] + acadoVariables.x[105];
acadoWorkspace.lbA[51] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[51] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[3] + acadoVariables.x[108];
acadoWorkspace.lbA[52] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[52] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[3] + acadoVariables.x[109];
acadoWorkspace.lbA[53] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[53] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[434]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[435]*acadoWorkspace.Dx0[3] + acadoVariables.x[112];
acadoWorkspace.lbA[54] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[54] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[436]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[437]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[438]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[439]*acadoWorkspace.Dx0[3] + acadoVariables.x[113];
acadoWorkspace.lbA[55] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[55] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[450]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[451]*acadoWorkspace.Dx0[3] + acadoVariables.x[116];
acadoWorkspace.lbA[56] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[56] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[452]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[453]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[454]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[455]*acadoWorkspace.Dx0[3] + acadoVariables.x[117];
acadoWorkspace.lbA[57] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[57] = (real_t)2.9999999999999999e-01 - tmp;
tmp = + acadoWorkspace.evGx[464]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[3] + acadoVariables.x[120];
acadoWorkspace.lbA[58] = (real_t)-1.0000000000000001e-01 - tmp;
acadoWorkspace.ubA[58] = (real_t)1.0000000000000001e-01 - tmp;
tmp = + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[3] + acadoVariables.x[121];
acadoWorkspace.lbA[59] = (real_t)-2.9999999999999999e-01 - tmp;
acadoWorkspace.ubA[59] = (real_t)2.9999999999999999e-01 - tmp;

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[20] = + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[21] = + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[22] = + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[23] = + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[24] = + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[25] = + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[26] = + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[27] = + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[28] = + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[29] = + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[3];
acadoWorkspace.lbA[60] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.lbA[61] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.lbA[62] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.lbA[63] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.lbA[64] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.lbA[65] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.lbA[66] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.lbA[67] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.lbA[68] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.lbA[69] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.lbA[70] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.lbA[71] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.lbA[72] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.lbA[73] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.lbA[74] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.lbA[75] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.lbA[76] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.lbA[77] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.lbA[78] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.lbA[79] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.lbA[80] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.lbA[81] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.lbA[82] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.lbA[83] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.lbA[84] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.lbA[85] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.lbA[86] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.lbA[87] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.lbA[88] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.lbA[89] -= acadoWorkspace.pacA01Dx0[29];

acadoWorkspace.ubA[60] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.ubA[61] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.ubA[62] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.ubA[63] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.ubA[64] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.ubA[65] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.ubA[66] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.ubA[67] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.ubA[68] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.ubA[69] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.ubA[70] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.ubA[71] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.ubA[72] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.ubA[73] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.ubA[74] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.ubA[75] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.ubA[76] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.ubA[77] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.ubA[78] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.ubA[79] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.ubA[80] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.ubA[81] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.ubA[82] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.ubA[83] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.ubA[84] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.ubA[85] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.ubA[86] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.ubA[87] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.ubA[88] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.ubA[89] -= acadoWorkspace.pacA01Dx0[29];

}

void acado_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];

acadoVariables.x[4] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3];
acadoVariables.x[5] += + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[3];
acadoVariables.x[6] += + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[3];
acadoVariables.x[7] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[3];
acadoVariables.x[8] += + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[3];
acadoVariables.x[9] += + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3];
acadoVariables.x[10] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[3];
acadoVariables.x[11] += + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[3];
acadoVariables.x[12] += + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[3];
acadoVariables.x[13] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[3];
acadoVariables.x[14] += + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3];
acadoVariables.x[15] += + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[3];
acadoVariables.x[16] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[3];
acadoVariables.x[17] += + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[3];
acadoVariables.x[18] += + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[3];
acadoVariables.x[19] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3];
acadoVariables.x[20] += + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[3];
acadoVariables.x[21] += + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[3];
acadoVariables.x[22] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[3];
acadoVariables.x[23] += + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[3];
acadoVariables.x[24] += + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[3];
acadoVariables.x[25] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[3];
acadoVariables.x[26] += + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[3];
acadoVariables.x[27] += + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[3];
acadoVariables.x[28] += + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[3];
acadoVariables.x[29] += + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[3];
acadoVariables.x[30] += + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[3];
acadoVariables.x[31] += + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[3];
acadoVariables.x[32] += + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[3];
acadoVariables.x[33] += + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[3];
acadoVariables.x[34] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3];
acadoVariables.x[35] += + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[3];
acadoVariables.x[36] += + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[3];
acadoVariables.x[37] += + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[3];
acadoVariables.x[38] += + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[3];
acadoVariables.x[39] += + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3];
acadoVariables.x[40] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[3];
acadoVariables.x[41] += + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[3];
acadoVariables.x[42] += + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[3];
acadoVariables.x[43] += + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[3];
acadoVariables.x[44] += + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[3];
acadoVariables.x[45] += + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[3];
acadoVariables.x[46] += + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[3];
acadoVariables.x[47] += + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[3];
acadoVariables.x[48] += + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[3];
acadoVariables.x[49] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3];
acadoVariables.x[50] += + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[3];
acadoVariables.x[51] += + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[3];
acadoVariables.x[52] += + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[3];
acadoVariables.x[53] += + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[3];
acadoVariables.x[54] += + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[3];
acadoVariables.x[55] += + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[3];
acadoVariables.x[56] += + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[3];
acadoVariables.x[57] += + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[3];
acadoVariables.x[58] += + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[3];
acadoVariables.x[59] += + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3];
acadoVariables.x[60] += + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[3];
acadoVariables.x[61] += + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[3];
acadoVariables.x[62] += + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[3];
acadoVariables.x[63] += + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[3];
acadoVariables.x[64] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3];
acadoVariables.x[65] += + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[3];
acadoVariables.x[66] += + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[3];
acadoVariables.x[67] += + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[3];
acadoVariables.x[68] += + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[3];
acadoVariables.x[69] += + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[3];
acadoVariables.x[70] += + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[3];
acadoVariables.x[71] += + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[3];
acadoVariables.x[72] += + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[3];
acadoVariables.x[73] += + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[3];
acadoVariables.x[74] += + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[3];
acadoVariables.x[75] += + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[3];
acadoVariables.x[76] += + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[3];
acadoVariables.x[77] += + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[3];
acadoVariables.x[78] += + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[3];
acadoVariables.x[79] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3];
acadoVariables.x[80] += + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[3];
acadoVariables.x[81] += + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[3];
acadoVariables.x[82] += + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[3];
acadoVariables.x[83] += + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[3];
acadoVariables.x[84] += + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[3];
acadoVariables.x[85] += + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[3];
acadoVariables.x[86] += + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[3];
acadoVariables.x[87] += + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[3];
acadoVariables.x[88] += + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[3];
acadoVariables.x[89] += + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[3];
acadoVariables.x[90] += + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[3];
acadoVariables.x[91] += + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[3];
acadoVariables.x[92] += + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[3];
acadoVariables.x[93] += + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[3];
acadoVariables.x[94] += + acadoWorkspace.evGx[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[363]*acadoWorkspace.Dx0[3];
acadoVariables.x[95] += + acadoWorkspace.evGx[364]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[3];
acadoVariables.x[96] += + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[3];
acadoVariables.x[97] += + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[3];
acadoVariables.x[98] += + acadoWorkspace.evGx[376]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[377]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[378]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[379]*acadoWorkspace.Dx0[3];
acadoVariables.x[99] += + acadoWorkspace.evGx[380]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[381]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[382]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[383]*acadoWorkspace.Dx0[3];
acadoVariables.x[100] += + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[3];
acadoVariables.x[101] += + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[3];
acadoVariables.x[102] += + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[3];
acadoVariables.x[103] += + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[3];
acadoVariables.x[104] += + acadoWorkspace.evGx[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[403]*acadoWorkspace.Dx0[3];
acadoVariables.x[105] += + acadoWorkspace.evGx[404]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[405]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[406]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[407]*acadoWorkspace.Dx0[3];
acadoVariables.x[106] += + acadoWorkspace.evGx[408]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[409]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[410]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[411]*acadoWorkspace.Dx0[3];
acadoVariables.x[107] += + acadoWorkspace.evGx[412]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[413]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[414]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[3];
acadoVariables.x[108] += + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[3];
acadoVariables.x[109] += + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[3];
acadoVariables.x[110] += + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[425]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[426]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[427]*acadoWorkspace.Dx0[3];
acadoVariables.x[111] += + acadoWorkspace.evGx[428]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[429]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[430]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[431]*acadoWorkspace.Dx0[3];
acadoVariables.x[112] += + acadoWorkspace.evGx[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[434]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[435]*acadoWorkspace.Dx0[3];
acadoVariables.x[113] += + acadoWorkspace.evGx[436]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[437]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[438]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[439]*acadoWorkspace.Dx0[3];
acadoVariables.x[114] += + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[3];
acadoVariables.x[115] += + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[3];
acadoVariables.x[116] += + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[450]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[451]*acadoWorkspace.Dx0[3];
acadoVariables.x[117] += + acadoWorkspace.evGx[452]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[453]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[454]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[455]*acadoWorkspace.Dx0[3];
acadoVariables.x[118] += + acadoWorkspace.evGx[456]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[457]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[458]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[459]*acadoWorkspace.Dx0[3];
acadoVariables.x[119] += + acadoWorkspace.evGx[460]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[461]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[462]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[463]*acadoWorkspace.Dx0[3];
acadoVariables.x[120] += + acadoWorkspace.evGx[464]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[3];
acadoVariables.x[121] += + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[3];
acadoVariables.x[122] += + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[475]*acadoWorkspace.Dx0[3];
acadoVariables.x[123] += + acadoWorkspace.evGx[476]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[477]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[478]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[479]*acadoWorkspace.Dx0[3];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multEDu( &(acadoWorkspace.E[ lRun3 * 8 ]), &(acadoWorkspace.x[ lRun2 * 2 ]), &(acadoVariables.x[ lRun1 * 4 + 4 ]) );
}
}
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 4];
acadoWorkspace.state[1] = acadoVariables.x[index * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 4 + 3];
acadoWorkspace.state[28] = acadoVariables.u[index * 2];
acadoWorkspace.state[29] = acadoVariables.u[index * 2 + 1];
acadoWorkspace.state[30] = acadoVariables.od[index * 4];
acadoWorkspace.state[31] = acadoVariables.od[index * 4 + 1];
acadoWorkspace.state[32] = acadoVariables.od[index * 4 + 2];
acadoWorkspace.state[33] = acadoVariables.od[index * 4 + 3];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 4 + 4] = acadoWorkspace.state[0];
acadoVariables.x[index * 4 + 5] = acadoWorkspace.state[1];
acadoVariables.x[index * 4 + 6] = acadoWorkspace.state[2];
acadoVariables.x[index * 4 + 7] = acadoWorkspace.state[3];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[120] = xEnd[0];
acadoVariables.x[121] = xEnd[1];
acadoVariables.x[122] = xEnd[2];
acadoVariables.x[123] = xEnd[3];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[120];
acadoWorkspace.state[1] = acadoVariables.x[121];
acadoWorkspace.state[2] = acadoVariables.x[122];
acadoWorkspace.state[3] = acadoVariables.x[123];
if (uEnd != 0)
{
acadoWorkspace.state[28] = uEnd[0];
acadoWorkspace.state[29] = uEnd[1];
}
else
{
acadoWorkspace.state[28] = acadoVariables.u[58];
acadoWorkspace.state[29] = acadoVariables.u[59];
}
acadoWorkspace.state[30] = acadoVariables.od[120];
acadoWorkspace.state[31] = acadoVariables.od[121];
acadoWorkspace.state[32] = acadoVariables.od[122];
acadoWorkspace.state[33] = acadoVariables.od[123];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[120] = acadoWorkspace.state[0];
acadoVariables.x[121] = acadoWorkspace.state[1];
acadoVariables.x[122] = acadoWorkspace.state[2];
acadoVariables.x[123] = acadoWorkspace.state[3];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 29; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[58] = uEnd[0];
acadoVariables.u[59] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59];
kkt = fabs( kkt );
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 90; ++index)
{
prd = acadoWorkspace.y[index + 60];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 5 */
real_t tmpDy[ 5 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 4];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 5] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 5];
acadoWorkspace.Dy[lRun1 * 5 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 5 + 1];
acadoWorkspace.Dy[lRun1 * 5 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 5 + 2];
acadoWorkspace.Dy[lRun1 * 5 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 5 + 3];
acadoWorkspace.Dy[lRun1 * 5 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 5 + 4];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acadoWorkspace.objValueIn[3] = acadoVariables.x[123];
acadoWorkspace.objValueIn[4] = acadoVariables.od[120];
acadoWorkspace.objValueIn[5] = acadoVariables.od[121];
acadoWorkspace.objValueIn[6] = acadoVariables.od[122];
acadoWorkspace.objValueIn[7] = acadoVariables.od[123];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 5];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 5 + 1];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 5 + 2];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 5 + 3];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 5 + 4];
objVal += + acadoWorkspace.Dy[lRun1 * 5]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 5 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 5 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 5 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 5 + 4]*tmpDy[4];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)5.0000000000000000e+00;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)5.0000000000000000e+00;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)5.0000000000000000e+00;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

