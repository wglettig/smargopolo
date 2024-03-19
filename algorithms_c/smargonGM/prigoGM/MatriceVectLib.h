#ifndef __MATRICEVECTLIB_H__
#define __MATRICEVECTLIB_H__

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
typedef double TypeMat3 [3][3];
typedef double TypeMat6 [6][6];
typedef double TypeMatH4 [4][4];

typedef double TypeVect9[9];
typedef double TypeMat9 [9][9];

#ifdef __cplusplus
extern "C" {
#endif

/*-------------         Prototypes       -----------------*/
// Fonction élevant un nombre au carré
double	Sqr(double value);
// Fonction imprimant une matrice 3x3
void		PrintMat3(TypeMat3 A, char Name[4]);
// Fonction imprimant une matrice homogène 4x4
void		PrintMatH4(TypeMatH4 A, char Name[4]);
// Fonction imprimant une matrice 6x6
void		PrintMat6(TypeMat6 A, char Name[4]);
// Fonction imprimant un vecteur 3x1
void		PrintVect3(TypeVect3 A, char Name[4]);
// Fonction imprimant un vecteur homogène 4x1
void		PrintVectH4(TypeVectH4 A, char Name[4]);
// Fonction imprimant un vecteur 6x1
void		PrintVect6(TypeVect6 A, char Name[4]);
// Fonction copiant un vecteur 3x1
void		CopyVect3(TypeVect3 Vi, TypeVect3 Vc);
// Fonction copiant un vecteur homogène 4x1
void		CopyVectH4(TypeVectH4 Vi, TypeVectH4 Vc);
// Fonction copiant un vecteur 6x1
void		CopyVect6(TypeVect6 Vi, TypeVect6 Vc);
// Fonction copiant une matrice 3x3
void		CopyMat3(TypeMat3 Mi, TypeMat3 Mc);
// Fonction copiant une matrice homogène 4x4
void		CopyMatH4(TypeMatH4 Mi, TypeMatH4 Mc);
// Fonction copiant une matrice 6x6
void		CopyMat6(TypeMat6 Mi, TypeMat6 Mc);
// Fonction transposant une matrice 3x3
void		TranspMat3(TypeMat3 A, TypeMat3 At);
// Fonction transposant une matrice 
// homogène 4x4 (transposée conventionnelle)
void		TranspMatH4(TypeMatH4 A, TypeMatH4 At);
// Fonction transposant une matrice 6x6
void TranspMat6(TypeMat6 A, TypeMat6 At);
// Fonction inversant une matrice homogèe 4x4
// inverse au sens homogène !=conventionnel
void		InvMatH4(TypeMatH4 A, TypeMatH4 Ai); 
// Fonction calculant le produit scalaire 
// de deux vecteurs 3x1
double	ProdScal3(TypeVect3 A, TypeVect3 B);
// Fonction calculant le produit scalaire 
// de deux vecteurs homogènes 4x1
double	ProdScalH4(TypeVectH4 A, TypeVectH4 B);
// Fonction calculant le produit scalaire 
// de deux vecteurs 6x1
double	ProdScal6(TypeVect6 A, TypeVect6 B);
// Fonction multipliant un vecteur 3x1 
// par un scalaire
void		MulVect3Scal(TypeVect3 A, double coeff, TypeVect3 R);
// Fonction multipliant un vecteur 
// homogène 4x1 par un scalaire
void		MulVectH4Scal(TypeVectH4 A, double coeff, TypeVectH4 R);
// Fonction multipliant un vecteur 6x1 
// par un scalaire
void		MulVect6Scal(TypeVect6 A, double coeff, TypeVect6 R);
// Fonction multipliant une matrice 3x3 
// par un scalaire
void		MulMat3Scal(TypeMat3 M, double coeff, TypeMat3 R);
// Fonction multipliant une matrice 6x6 
// par un scalaire
void		MulMat6Scal(TypeMat6 M, double coeff, TypeMat6 R);
// Fonction multipliant un vecteur 3x1 
// par une matrice 3x3
void		MulMat3Vect3(TypeMat3 M, TypeVect3 A, TypeVect3 R);
// Fonction multipliant un vecteur 6x1 
// par une matrice 6x6
void		MulMat6Vect6(TypeMat6 M, TypeVect6 A, TypeVect6 R);
// Fonction multipliant un vecteur 3x1 
// par une matrice homogène 4x4
void		MulMatH4Vect3(TypeMatH4 M, TypeVect3 A, TypeVect3 R);
// Fonction multipliant un vecteur 
// homogène 4x1 par une matrice homogène 4x4
void		MulMatH4VectH4(TypeMatH4 M, TypeVectH4 A, TypeVectH4 R);
// Fonction multipliant une matrice 3x3 
// par une matrice 3x3
void		MulMatMat3(TypeMat3 M1, TypeMat3 M2, TypeMat3 R);
// Fonction multipliant une matrice 
// homogène 4x4 par une matrice homogène 4x4
void		MulMatMatH4(TypeMatH4 M1, TypeMatH4 M2, TypeMatH4 R); 
// Fonction multipliant une matrice 6x6 
// par une matrice 6x6
void		MulMatMat6(TypeMat6 M1, TypeMat6 M2, TypeMat6 R);
// Fonction sommant deux vecteurs 3x1 
void		SomVectVect3(TypeVect3 A, TypeVect3 B, TypeVect3 R);
// Fonction soustrayant deux vecteurs 3x1
void		SubVectVect3(TypeVect3 A, TypeVect3 B, TypeVect3 R);
// Fonction sommant deux vecteurs 
// homogènes 4x1 
void		SomVectVectH4(TypeVectH4 A, TypeVectH4 B, TypeVectH4 R);
// Fonction soustrayant deux vecteurs 
// homogènes 4x1 
void		SubVectVectH4(TypeVectH4 A, TypeVectH4 B, TypeVectH4 R);
// Fonction sommant deux vecteurs 6x1 
void		SomVectVect6(TypeVect6 A, TypeVect6 B, TypeVect6 R);
// Fonction soustrayant deux vecteurs 6x1
void		SubVectVect6(TypeVect6 A, TypeVect6 B, TypeVect6 R);
// Fonction sommant deux matrices 3x3 
void		SomMatMat3(TypeMat3 A, TypeMat3 B, TypeMat3 R);
// Fonction soustrayant deux matrices 3x3 
void		SubMatMat3(TypeMat3 A, TypeMat3 B, TypeMat3 R);
// Fonction sommant deux matrices 6x6 
void		SomMatMat6(TypeMat6 A, TypeMat6 B, TypeMat6 R);
// Fonction soustrayant deux matrices 6x6 
void		SubMatMat6(TypeMat6 A, TypeMat6 B, TypeMat6 R);
// Fonction initialisant un vecteur 3x1 
void		InitialVect3(TypeVect3 A);
// Fonction initialisant un vecteur 
// homogène 4x1 
void		InitialVectH4(TypeVectH4 A);
// Fonction initialisant un vecteur 6x1
void		InitialVect6(TypeVect6 A);
// Fonction initialisant une matrice 3x3 
void		InitialMat3(TypeMat3 A);
// Fonction initialisant une matrice 6x6
void		InitialMat6(TypeMat6 A);
// Fonction créant une matrice identité 3x3 
void		IdMat3(TypeMat3 A);
// Fonction créant une matrice 
// homogène identité 4x4
void		IdMatH4(TypeMatH4 A);
// Fonction créant une matrice identité 6x6 
void		IdMat6(TypeMat6 A);
// Fonction calculant la norme au carré 
// d'un vecteur 3x1
double	NormeCarreVect3(TypeVect3 A);
// Fonction calculant la norme au carré 
// d'un vecteur homogène 4x1
double	NormeCarreVectH4(TypeVectH4 A);
// Fonction créant un vecteur homogène 4x1  
// à partir d'un vecteur position 3x1
void		MakeVectH4Pos(TypeVect3 Pos, TypeVectH4 V);
// Fonction créant une matrice de rotation 3x3  
// à partir d'un vecteur de position 
// angulaire 3x1
void		MakeMat3Rot(TypeVect3 Ang, TypeMat3 R);
// Fonction créant une matrice homogène 4x4  
// à partir d'un vecteur position angulaire 
// 3x1 et d'un vecteur de position 3x1
void		MakeMatH4AngPos(TypeVect3 Ang, TypeVect3 Pos, TypeMatH4 H); 
// Fonction créant une matrice homogène 4x4  
// à partir d'une matrice de rotation 3x3 
// et d'un vecteur de position 3x1
void		MakeMatH4RotPos(TypeMat3 R, TypeVect3 Pos, TypeMatH4 H);
// Fonction créant une matrice homogène 4x4  
// à partir d'un vecteur de position angulaire 
// inverse 3x1 
// et d'un vecteur de position 3x1
void		MakeMatH4InvAngPos(TypeVect3 Ang, TypeVect3 Pos, TypeMatH4 H);
// Fonction créant une matrice homogène 4x4  
// à partir d'une matrice de rotation inverse
// 3x3 et d'un vecteur de position 3x1
void		MakeMatH4InvRotPos(TypeMat3 R, TypeVect3 Pos, TypeMatH4 H);
// Fonction créant une matrice homogène 4x4  
// à partir d'un vecteur de 
// position généralisée 6x1
void		MakeMatH4PosGen(TypeVect6 Pos,TypeMatH4 H);
// Fonction créant une matrice 6x6 liant 
// la vitesse d'un point C attaché au point B 
// à la vitesse du point B, 
// le tout exprimé dans un référentiel A
void		MakeMat6RotPVit(TypeMat3 R,TypeVect6 Pos, TypeMat6 O);
// Fonction créant une matrice liant 
// la force en un point C à la force 
// équivalente en un point B, 
// le tout exprimé dans un référentiel A
void		MakeMat6RotPForce(TypeMat3 R,TypeVect6 Pos, TypeMat6 O);
// Fonction créant une matrice 6x6 contenant
// deux matrices de rotation 3x3 
// sur la diagonale 
void		MakeMat6Rot(TypeMat3 R,TypeMat6 O);
// Fonction créant une matrice de gain 6x6 
// ayant comme diagonale les valeurs 
// fournies en entrée  
void		MakeMat6Gain(double gain_1,double gain_2,double gain_3,double gain_4,double gain_5,double gain_6,TypeMat6 R);
// Fonction créant un vecteur de gain 6x1 
// à partir des valeurs fournies en entrée
void		MakeVect6Gain(double gain_1,double gain_2,double gain_3,double gain_4,double gain_5,double gain_6,TypeVect6 R);
// Fonction extrayant la position 
// angulaire dans un vecteur 3x1
// à partir d'une matrice de rotation 3x3
void		ExtractAngMat3(TypeMat3 R, TypeVect3 Ang);
// Fonction extrayant la position 
// angulaire dans un vecteur 3x1
// à partir d'une matrice homogène 4x4
void		ExtractAngMatH4(TypeMatH4 H, TypeVect3 Ang); 
// Fonction extrayant la position 
// dans un vecteur 3x1
// à partir d'un vecteur homogène 4x1
void		ExtractPosVectH4(TypeVectH4 V, TypeVect3 Pos);
// Fonction extrayant la position 
// dans un vecteur 3x1
// à partir d'une matrice homogène 4x4
void		ExtractPosMatH4(TypeMatH4 H, TypeVect3 Pos); 
// Fonction extrayant la matrice 
// de rotation 3x3
// à partir d'une matrice homogène 4x4
void		ExtractRotMatH4(TypeMatH4 H, TypeMat3 R);
// Fonction extrayant la position 
// généralisée dans un vecteur 6x1
// à partir d'une matrice homogène 4x4
void		ExtractPosGenMatH4(TypeMatH4 H,TypeVect6 Pos);
// Fonction calculant l'inverse d'une matrice
// 6x6. Le résultat est dans la matrice A.
void		InvMat6(TypeMat6 A, TypeMat6 B);
/*--------------------------------------------------------*/

// Fonction calculant la norme au carré 
// d'un vecteur nx1:
double NormeCarreVect(double* V, int n);


/*------ LU Solve for n x n Equation systems  Ax = b -----*/
////////////////////////////////////////////////////////////////////////////////
//  int Doolittle_LU_Decomposition_with_Pivoting(double *A, int pivot[],      //
//                                                                    int n)  //
//                                                                            //
//  Description:                                                              //
//     This routine uses Doolittle's method to decompose the n x n matrix A   //
//     into a unit lower triangular matrix L and an upper triangular matrix U //
//     such that A = LU.                                                      //
//     The matrices L and U replace the matrix A so that the original matrix  //
//     A is destroyed.  Note!  In Doolittle's method the diagonal elements of //
//     L are 1 and are not stored.                                            //
//     The LU decomposition is convenient when one needs to solve the linear  //
//     equation Ax = B for the vector x while the matrix A is fixed and the   //
//     vector B is varied.  The routine for solving the linear system Ax = B  //
//     after performing the LU decomposition for A is                         //
//                      Doolittle_LU_with_Pivoting_Solve.                     //
//                                                                            //
//     The Doolittle method with partial pivoting is:  Determine the pivot    //
//     row and interchange the current row with the pivot row, then assuming  //
//     that row k is the current row, k = 0, ..., n - 1 evaluate in order the //
//     following pair of expressions                                          //
//       U[k][j] = A[k][j] - (L[k][0]*U[0][j] + ... + L[k][k-1]*U[k-1][j])    //
//                                 for j = k, k+1, ... , n-1                  //
//       L[i][k] = (A[i][k] - (L[i][0]*U[0][k] + . + L[i][k-1]*U[k-1][k]))    //
//                          / U[k][k]                                         //
//                                 for i = k+1, ... , n-1.                    //
//       The matrix U forms the upper triangular matrix, and the matrix L     //
//       forms the lower triangular matrix.                                   //
//                                                                            //
//  Arguments:                                                                //
//     double *A       Pointer to the first element of the matrix A[n][n].    //
//     int    pivot[]  The i-th element is the pivot row interchanged with    //
//                     row i.                                                 //
//     int     n       The number of rows or columns of the matrix A.         //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix A is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double A[N][N];                                                        //
//     int    pivot[N];                                                       //
//                                                                            //
//     (your code to create matrix A)                                         //
//     err = Doolittle_LU_Decomposition_with_Pivoting(&A[0][0], pivot, N);    //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else { printf(" The LU decomposition of A is \n");                     //
//           ...                                                              //
////////////////////////////////////////////////////////////////////////////////
//                      
int Doolittle_LU_Decomposition_with_Pivoting(double *A, int pivot[], int n);

////////////////////////////////////////////////////////////////////////////////
//  int Doolittle_LU_with_Pivoting_Solve(double *LU, double *B, int pivot[],  //
//                                       double *x,  int n)                   //
//                                                                            //
//  Description:                                                              //
//     This routine uses Doolittle's method to solve the linear equation      //
//     Ax = B.  This routine is called after the matrix A has been decomposed //
//     into a product of a unit lower triangular matrix L and an upper        //
//     triangular matrix U with pivoting.  The argument LU is a pointer to the//
//     matrix the subdiagonal part of which is L and the superdiagonal        //
//     together with the diagonal part is U. (The diagonal part of L is 1 and //
//     is not stored.)   The matrix A = LU.                                   //
//     The solution proceeds by solving the linear equation Ly = B for y and  //
//     subsequently solving the linear equation Ux = y for x.                 //
//                                                                            //
//  Arguments:                                                                //
//     double *LU      Pointer to the first element of the matrix whose       //
//                     elements form the lower and upper triangular matrix    //
//                     factors of A.                                          //
//     double *B       Pointer to the column vector, (n x 1) matrix, B.       //
//     int    pivot[]  The i-th element is the pivot row interchanged with    //
//                     row i.                                                 //
//     double *x       Solution to the equation Ax = B.                       //
//     int     n       The number of rows or columns of the matrix LU.        //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix A is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double A[N][N], B[N], x[N];                                            //
//     int    pivot[N];                                                       //
//                                                                            //
//     (your code to create matrix A and column vector B)                     //
//     err = Doolittle_LU_Decomposition_with_Pivoting(&A[0][0], pivot,  N);   //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else {                                                                 //
//        err = Doolittle_LU_with_Pivoting_Solve(&A[0][0], B, pivot, x, N);   //
//        if (err < 0) printf(" Matrix A is singular\n");                     //
//        else printf(" The solution is \n");                                 //
//           ...                                                              //
//     }                                                                      //
////////////////////////////////////////////////////////////////////////////////
//                  
int Doolittle_LU_with_Pivoting_Solve(double *A, double B[], int pivot[],double x[], int n);





#ifdef __cplusplus
}
#endif

#endif


