#include "kinematics.h"
#include "elog.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#define TAG "Kinematics"

#define PI 3.141592653589793f
#define DEG_TO_RAD(deg) ((deg) * 0.01745329251994329547f)
#define RAD_TO_DEG(rad) ((rad) * 57.29577951308232286465f)

static const double L1 = 80.0f;
static const double L2 = 65.0f;
static const double L3 = 20.0f;
// static const double _L1 = 75.0f;
// static const double L4 = 32.0f;
static const double L5 = 81.54140052758476f; // sqrt(_L1 * _L1 + L4 * L4);
static const double L8 = 15.0f;
static const double L9 = 73.0f;
// static const double R14 = PI / 2;
static const double R15 = 0.4032814817188361f; // atan(L4 / _L1);

double I1_XZ_to_L7(double X, double Z) {
	return sqrt(pow(X, 2) + pow(Z, 2));
}

double I2_L7_to_R17(double L7) {
	return acos((pow(L1, 2) + pow(L7, 2) - pow(L2, 2)) / (2 * L1 * L7));
}

double I3_L7_to_R12(double L7) {
	double T = (pow(L1, 2) + pow(L2, 2) - pow(L7, 2)) / (2 * L1 * L2);
	return acos(T);
}

double I4_XZ_to_RX7(double X, double Z) {
	double K = Z / X;
	if (X >= 0 && Z >= 0) {
		return atan(K);
	} else if (X < 0 && Z >= 0) {
		return PI + atan(K);
	} else if (X < 0 && Z < 0) {
		return PI + atan(K);
	}
	return 0.0;
}

double I5_R17R7X_to_RS1(double R17, double R7X) {
	return R7X - R17;
}

double I6_R12_to_R35(double R12) {
	return PI - R12 - R15;
}

double I7_R35_to_L6(double R35) {
	double L6 = sqrt(pow(L3, 2) + pow(L5, 2) - 2 * L3 * L5 * cos(R35));
	if (L6 > L8 + L9) {
		fprintf(stderr, "Error: L6[%f] > L8 + L9[%f]\n", L6, L8 + L9);
	}
	return L6;
}

double I8_L6_to_RS2(double L6) {
	double T = (L6 * L6 + L8 * L8 - L9 * L9) / (2 * L6 * L8);
	return acos(T);
}

Data caculateFromXZ(double X, double Z) {
	Data data;
	data.L7 = I1_XZ_to_L7(X, Z);
	data.R17 = I2_L7_to_R17(data.L7);
	data.R12 = I3_L7_to_R12(data.L7);
	data.R7X = I4_XZ_to_RX7(X, Z);
	data.RS1 = I5_R17R7X_to_RS1(data.R17, data.R7X);
	data.R35 = I6_R12_to_R35(data.R12);
	data.L6 = I7_R35_to_L6(data.R35);
	data.RS2 = I8_L6_to_RS2(data.L6);
	data.AS1 = RAD_TO_DEG(data.RS1);
	data.AS2 = RAD_TO_DEG(data.RS2);
	data.R13 = PI - data.R12;
	data.X = X;
	data.Z = Z;
	return data;
}

void logComparison(bool eq, const char* tag, double val1, double val2) {
	if (eq) {
		elog_d(TAG,"| %5s | %20.10f | %20.10f | %20.10f |", tag, val1, val2, fabs(val1 - val2));
	} else {
		elog_e(TAG,"| %5s | %20.10f | %20.10f | %20.10f | XXX", tag, val1, val2, fabs(val1 - val2));
	}
}

bool dataCompare(const Data* d1, const Data* d2) {
	const double angle_error = 0.001;
	const double radian_error = DEG_TO_RAD(angle_error);
	const double length_error = 0.001;

	bool AS1_eq = fabs(d1->AS1 - d2->AS1) < angle_error;
	bool AS2_eq = fabs(d1->AS2 - d2->AS2) < angle_error;
	bool RS1_eq = fabs(d1->RS1 - d2->RS1) < radian_error;
	bool RS2_eq = fabs(d1->RS2 - d2->RS2) < radian_error;
	bool L6_eq  = fabs(d1->L6  - d2->L6)  < length_error;
	bool L7_eq  = fabs(d1->L7  - d2->L7)  < length_error;
	bool R12_eq = fabs(d1->R12 - d2->R12) < radian_error;
	bool R13_eq = fabs(d1->R13 - d2->R13) < radian_error;
	bool R17_eq = fabs(d1->R17 - d2->R17) < radian_error;
	bool R35_eq = fabs(d1->R35 - d2->R35) < radian_error;
	bool R7X_eq = fabs(d1->R7X - d2->R7X) < radian_error;
	bool X_eq   = fabs(d1->X   - d2->X)   < length_error;
	bool Z_eq   = fabs(d1->Z   - d2->Z)   < length_error;

	bool all_eq = AS1_eq && AS2_eq && RS1_eq && RS2_eq && L6_eq && L7_eq &&
	              R12_eq && R13_eq && R17_eq && R35_eq && R7X_eq && X_eq && Z_eq;


	if (!all_eq) {
		elog_e(TAG, "Data comparison failed!");
	}else {
		elog_i(TAG, "Data comparison passed!");
	}

	logComparison(AS1_eq, "AS1", d1->AS1, d2->AS1);
	logComparison(AS2_eq, "AS2", d1->AS2, d2->AS2);
	logComparison(RS1_eq, "RS1", d1->RS1, d2->RS1);
	logComparison(RS2_eq, "RS2", d1->RS2, d2->RS2);
	logComparison(L6_eq,  "L6",  d1->L6,  d2->L6);
	logComparison(L7_eq,  "L7",  d1->L7,  d2->L7);
	logComparison(R12_eq, "R12", d1->R12, d2->R12);
	logComparison(R13_eq, "R13", d1->R13, d2->R13);
	logComparison(R17_eq, "R17", d1->R17, d2->R17);
	logComparison(R35_eq, "R35", d1->R35, d2->R35);
	logComparison(R7X_eq, "R7X", d1->R7X, d2->R7X);
	logComparison(X_eq,   "X",   d1->X,   d2->X);
	logComparison(Z_eq,   "Z",   d1->Z,   d2->Z);

	return all_eq;
}

void dataElog(const char* data_tag, const Data* data) {
	printf("%s\n"
	"| AS1   : %7.3f  | AS2   : %7.3f  |\n"
	"| RS1   : %7.3f  | RS2   : %7.3f  |\n"
	"| L6    : %7.3f  | L7    : %7.3f  |\n"
	"| R12   : %7.3f  | R13   : %7.3f  |\n"
	"| R35   : %7.3f  | R7X   : %7.3f  |\n"
	"| X     : %7.3f  | Z     : %7.3f  |\n",
	  data_tag,
		data->AS1, data->AS2,
		data->RS1, data->RS2,
		data->L6, data->L7,
		data->R12, data->R13,
		data->R35, data->R7X,
		data->X, data->Z);
}