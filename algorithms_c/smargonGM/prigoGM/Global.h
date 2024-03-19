//Septembre 2003
//Modification: Marc Kunze

#ifndef __GLOBAL_H__
#define __GLOBAL_H__

/*-------------   Constant definitions  -----------------*/

#define degre 	6 

#define axeX	0
#define axeY	1
#define axeZ	2
#define alpha	3
#define beta	4
#define gamma	5

// Constantes mathématiques
#define Pi 3.1415926535898
#define EPSILON_DIV_ZERO_DOUBLE 1e-12

/*-------------     Type definitions     -----------------*/
typedef double TypeVect[degre];
typedef double TypeVect6[6];
typedef double TypeVect3[3];
typedef double TypeVectH4[4];
typedef double TypeMat[degre][degre];
typedef double TypeMat3[3][3];
typedef double TypeMat6[6][6];
typedef double TypeMatH4[4][4];
/*--------------------------------------------------------*/

#endif
