#include <inttypes.h>

#define PI  3.141592f

#define sin_a0  0.707106f
#define sin_a2  -0.872348f
#define sin_a4  0.179251f
#define sin_a6  -0.014271f
#define sin_b1  -1.110670f
#define sin_b3  0.456158f
#define sin_b5  -0.053910f

#define asin_a0 0.318309f
#define asin_a2 -0.518287f
#define asin_a4 0.222375f
#define asin_a6 -0.016850f
#define asin_b0 2*0.5f
#define asin_b2 2*-0.897458f
#define asin_b4 2*0.461381f
#define asin_b6 2*-0.058377f

#define atan_a0 0.25f
#define atan_a1 0.636117f
#define atan_a3 -0.817719f
#define atan_a5 1.48697f
#define atan_a7 -1.57588f

int check_quadrant(float angle)
{
    int quadrant;
    if(angle<PI/2) quadrant=1;
    else if(angle<PI) quadrant=2;
    else if(angle<3*PI/2) quadrant=3;
    else quadrant=4;

    return quadrant;
}

// trigonometric functions approximations using polynomials

float fast_sin(float a)
{
    float A,B;
    int quadrant,sign=1; // 1 means positive, -1 means negative

    while(a<0) a+=2*PI;

    if(a>2*PI) a = a - ( (int)(a/(2*PI)) * 2*PI);

    quadrant = check_quadrant(a);

    if(quadrant==1) asm volatile ("nop");
    else if(quadrant==2) a=PI-a;
    else if(quadrant==3)
    {
        a=a-PI;
        sign=-1;
    }

    else if(quadrant==4)
    {
        a=2*PI-a;
        sign=-1;
    }

    a=(2/PI)*a - 0.5f;

    A=sin_a0+sin_a2*a*a+sin_a4*a*a*a*a+sin_a6*a*a*a*a*a*a;
    B=sin_b1*a+sin_b3*a*a*a+sin_b5*a*a*a*a*a;

    return (A-B)*sign;
}

float fast_cos(float a)
{
    return fast_sin(PI/2+a);
}

float fast_asin(float a)
{
    float A,B;
    A=asin_a0+asin_a2*a*a+asin_a4*a*a*a*a+asin_a6*a*a*a*a*a*a;
    B=asin_b0+asin_b2*a*a+asin_b4*a*a*a*a+asin_b6*a*a*a*a*a*a;
    return PI*a*(A/B);
}

float fast_acos(float a)
{
    return (PI/2 - fast_asin(a));
}

float fast_atan(float a)
{
	int sign=1;
	if(a<0)
	{
		sign=-1;
	    a=-a;
	}

    a=(a-1)/(2*a+2);
    return sign*PI*(atan_a0+atan_a1*a+atan_a3*a*a*a+atan_a5*a*a*a*a*a+atan_a7*a*a*a*a*a*a*a);
}

