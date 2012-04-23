/*
	File:		LibVLd.cc
	
	Purpose:	Compiles all code necessary for VLd.h.
	
	Author:		Andrew Willmott
*/


#define VL_V_REAL Double
#define VL_V_SUFF(X) X ## d
#define VL_M_REAL Double
#define VL_M_SUFF(X) X ## d

#include "Vec2.cc"
#include "Vec3.cc"
#include "Vec4.cc"
#include "Vec.cc"
#include "SparseVec.cc"
#include "SubVec.cc"
#include "SubSVec.cc"

#include "Mat2.cc"
#include "Mat3.cc"
#include "Mat4.cc"
#include "Mat.cc"
#include "SparseMat.cc"
#include "Solve.cc"
#include "Factor.cc"

#include "SubMat.cc"
#include "SubSMat.cc"
