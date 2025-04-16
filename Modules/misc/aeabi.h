#pragma once

#ifdef __cplusplus
extern "C" {
#endif

extern double             __aeabi_dadd(double a, double b);
extern double             __aeabi_dsub(double a, double b);
extern double             __aeabi_dmul(double a, double b);
extern double             __aeabi_ddiv(double a, double b);
extern float              __aeabi_fadd(float a, float b);
extern float              __aeabi_fsub(float a, float b);
extern float              __aeabi_fmul(float a, float b);
extern float              __aeabi_fdiv(float a, float b);

extern double             __aeabi_i2d(int a);
extern double             __aeabi_ui2d(unsigned int a);
extern double             __aeabi_l2d(long long a);
extern double             __aeabi_ul2d(unsigned long long a);

extern float              __aeabi_i2f(int a);
// extern float              __aeabi_ui2f(unsigned int a); //!
extern float              __aeabi_l2f(long long a);
// extern float              __aeabi_ul2f(unsigned long long a); //!

extern int                __aeabi_d2iz(double a);
extern unsigned int       __aeabi_d2uiz(double a);
extern long long          __aeabi_d2lz(double a);
extern unsigned long long __aeabi_d2ulz(double a);

extern int                __aeabi_f2iz(float a);
extern unsigned int       __aeabi_f2uiz(float a);
extern long long          __aeabi_f2lz(float a);
extern unsigned long long __aeabi_f2ulz(float a);

extern float              __aeabi_d2f(double a);
extern double             __aeabi_f2d(double a);

#ifdef __cplusplus
}
#endif
