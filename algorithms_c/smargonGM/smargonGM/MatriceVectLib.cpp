//********************************************
// Fonctions vectorielles et matricielles
// destinées à la commande de M. Bouri
// 
// Auteur: Patrick Helmer
// Edité par: Marc Kunze
// Edité par: Wayne Glettig
//
// Un vecteur ou une matrice débute 
// par l'indice 0
//********************************************


#include <stdio.h>
#include <math.h>
#include "MatriceVectLib.h"


//********************************************
// Fonctions algébriques
//********************************************

/**
* Fonction élevant un nombre au carré
*/
double Sqr(double value)
{
	return(value*value);
}


//********************************************
// Fonctions d'impression vectorielles et 
// matricielles
//********************************************

/**
* Fonction imprimant une matrice 3x3
*/
void PrintMat3(TypeMat3 A, char Name[4])
{
	int i, j;
	
	printf("#### %s #### \n", Name);
	for (i=0; i<3; i++)
	{
		for (j=0; j<3; j++)
			printf("    %12.9f", A[i][j]);
		printf("\n");
	}
}


/**
* Fonction imprimant une matrice homogène 4x4
*/
void PrintMatH4(TypeMatH4 A, char Name[4])
{
	int i, j;
	
	printf("#### %s #### \n", Name);
	for (i=0; i<4; i++)
	{
		for (j=0; j<4; j++)
			printf("    %0.6f", A[i][j]);
		printf("\n");
	}
}


/**
* Fonction imprimant une matrice 6x6
*/
void PrintMat6(TypeMat6 A, char Name[4])
{
	int i, j;
	
	printf("#### %s #### \n", Name);
	for (i=0; i<6; i++)
	{
		for (j=0; j<6; j++)
			printf("  %10.2e", A[i][j]);
		printf("\n");
	}
}


/**
* Fonction imprimant un vecteur 3x1
*/
void PrintVect3(TypeVect3 A, char Name[4])
{
	int i;
	
	printf("#### %s #### \n", Name);
	for (i=0; i<3; i++)
		printf("%0.9f\n", A[i]);
}


/**
* Fonction imprimant un vecteur homogène 4x1
*/
void PrintVectH4(TypeVectH4 A, char Name[4])
{
	int i;
	
	printf("#### %s #### \n", Name);
	for (i=0; i<4; i++)
		printf("%0.9f\n", A[i]);
}


/**
* Fonction imprimant un vecteur 6x1
*/
void PrintVect6(TypeVect6 A, char Name[4])
{
	int i;
	
	printf("#### %s #### \n", Name);
	for (i=0; i<6; i++)
		printf("%0.9f\n", A[i]);
}


//********************************************
// Fonctions de manipulations vect et mat
//********************************************

/**
* Fonction copiant un vecteur 3x1
*/
void CopyVect3(TypeVect3 Vi, TypeVect3 Vc)
{
	int i;
	
	for (i=0; i<3; i++)
		Vc[i]=Vi[i];
}


/**
* Fonction copiant un vecteur homogène 4x1
*/
void CopyVectH4(TypeVectH4 Vi, TypeVectH4 Vc)
{
	int i;
	
	for (i=0; i<4; i++)
		Vc[i]=Vi[i];
}


/**
* Fonction copiant un vecteur 6x1
*/
void CopyVect6(TypeVect6 Vi, TypeVect6 Vc)
{
	int i;

	for (i=0; i<6; i++)
		Vc[i]=Vi[i];
}


/**
* Fonction copiant une matrice 3x3
*/
void CopyMat3(TypeMat3 Mi, TypeMat3 Mc)
{
	int i, j;

	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			Mc[i][j]=Mi[i][j];
}


/**
* Fonction copiant une matrice homogène 4x4
*/
void CopyMatH4(TypeMatH4 Mi, TypeMatH4 Mc)
{
	int i, j;

	for (i=0; i<4; i++)
		for (j=0; j<4; j++)
			Mc[i][j]=Mi[i][j];
}


/**
* Fonction copiant une matrice 6x6
*/
void CopyMat6(TypeMat6 Mi, TypeMat6 Mc)
{
	int i, j;

	for (i=0; i<6; i++)
		for (j=0; j<6; j++)
			Mc[i][j]=Mi[i][j];
}


/**
* Fonction transposant une matrice 3x3
*/
void TranspMat3(TypeMat3 A, TypeMat3 At)
{
	int		i, j;
	TypeMat3	Atrep;

	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			Atrep[j][i]=A[i][j];

	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			At[i][j]=Atrep[i][j];
}


/**
* Fonction transposant une matrice 
* homogène 4x4 (transposée conventionnelle)
*/
void TranspMatH4(TypeMatH4 A, TypeMatH4 At)
{
	int		i, j;
	TypeMatH4	Atrep;

	for (i=0; i<4; i++)
		for (j=0; j<4; j++)
			Atrep[j][i]=A[i][j];

	for (i=0; i<4; i++)
		for (j=0; j<4; j++)
			At[i][j]=Atrep[i][j];
}


/**
* Fonction transposant une matrice 6x6
*/
void TranspMat6(TypeMat6 A, TypeMat6 At)
{
	int		i, j;
	TypeMat6	Atrep;

	for (i=0; i<6; i++)
		for (j=0; j<6; j++)
			Atrep[j][i]=A[i][j];

	for (i=0; i<6; i++)
		for (j=0; j<6; j++)
			At[i][j]=Atrep[i][j];
}


/**
* Fonction inversant une matrice homogèe 4x4
* inverse au sens homogène !=conventionnel
*/ 
void InvMatH4(TypeMatH4 A, TypeMatH4 Ai)
{
	TypeMat3	R;
	TypeVect3	P;

	ExtractRotMatH4(A, R);
	TranspMat3(R, R);
	ExtractPosMatH4(A, P);
	MulMat3Vect3(R, P, P);
	MulVect3Scal(P, -1.0f, P);
	MakeMatH4RotPos(R, P, Ai);
}


/**
* Fonction calculant le produit scalaire 
* de deux vecteurs 3x1
*/
double ProdScal3(TypeVect3 A, TypeVect3 B)
{
	int		i;
	double	r;

	r=0;
	for (i=0; i<3; i++)
		r=r+A[i]*B[i];
	return r;
}


/**
* Fonction calculant le produit scalaire 
* de deux vecteurs homogènes 4x1
*/
double ProdScalH4(TypeVectH4 A, TypeVectH4 B)
{
	int		i;
	double	r;

	r=0;
	for (i=0; i<3; i++)
		r=r+A[i]*B[i];
	return r;
}


/**
* Fonction calculant le produit scalaire 
* de deux vecteurs 6x1
*/
double ProdScal6(TypeVect6 A, TypeVect6 B)
{
	int		i;
	double	r;

	r=0;
	for (i=0; i<6; i++)
		r=r+A[i]*B[i];
	return r;
}


/**
* Fonction multipliant un vecteur 3x1 
* par un scalaire
*/
void MulVect3Scal(TypeVect3 A, double coeff, TypeVect3 R)
{
	int  i;

	for (i=0; i<3; i++)
		R[i]=A[i]*coeff;
}


/**
* Fonction multipliant un vecteur 
* homogène 4x1 par un scalaire
*/
void MulVectH4Scal(TypeVectH4 A, double coeff, TypeVectH4 R)
{
	int  i;
	
	for (i=0; i<3; i++)
		R[i]=A[i]*coeff;
	R[3]=1;
}


/**
* Fonction multipliant un vecteur 6x1 
* par un scalaire
*/
void MulVect6Scal(TypeVect6 A, double coeff, TypeVect6 R)
{
	int  i;

	for (i=0; i<6; i++)
		R[i]=A[i]*coeff;
}


/**
* Fonction multipliant une matrice 3x3 
* par un scalaire
*/
void MulMat3Scal(TypeMat3 M, double coeff, TypeMat3 R)
{
	int  i, j;
	
	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			R[i][j]=M[i][j]*coeff;
}

/**
* Fonction multipliant une matrice 6x6 
* par un scalaire
*/
void MulMat6Scal(TypeMat6 M, double coeff, TypeMat6 R)
{
	int  i, j;
	
	for (i=0; i<6; i++)
		for (j=0; j<6; j++)
			R[i][j]=M[i][j]*coeff;
}

/**
* Fonction multipliant un vecteur 3x1 
* par une matrice 3x3
*/
void MulMat3Vect3(TypeMat3 M, TypeVect3 A, TypeVect3 R)
{
	int		i, j;
	TypeVect3	Rep;
	
	for (i=0; i<3; i++)
	{
		Rep[i]=0;
		for (j=0; j<3; j++)
			Rep[i]=Rep[i]+M[i][j]*A[j];
	}

	for (i=0; i<3; i++)
		R[i]=Rep[i];
}


/**
* Fonction multipliant un vecteur 6x1 
* par une matrice 6x6
*/
void MulMat6Vect6(TypeMat6 M, TypeVect6 A, TypeVect6 R)
{
	int		i, j;
	TypeVect6	Rep;
	
	for (i=0; i<6; i++)
	{
		Rep[i]=0;
		for (j=0; j<6; j++)
			Rep[i]=Rep[i]+M[i][j]*A[j];
	}

	for (i=0; i<6; i++)
		R[i]=Rep[i];
}


/**
* Fonction multipliant un vecteur 3x1 
* par une matrice homogène 4x4
*/
void MulMatH4Vect3(TypeMatH4 M, TypeVect3 A, TypeVect3 R)
{
	int		i, j;
	TypeVect3	Rep;
	
	for (i=0; i<3; i++)
	{
		Rep[i]=0;
		for (j=0; j<3; j++)
			Rep[i]=Rep[i]+M[i][j]*A[j];
		Rep[i]=Rep[i]+M[i][3];
	}

	for (i=0; i<3; i++)
		R[i]=Rep[i];
}


/**
* Fonction multipliant un vecteur 
* homogène 4x1 par une matrice homogène 4x4
*/
void MulMatH4VectH4(TypeMatH4 M, TypeVectH4 A, TypeVectH4 R)
{
	int			i, j;
	TypeVectH4	Rep;
	
	for (i=0; i<4; i++)
	{
		Rep[i]=0;
		for (j=0; j<4; j++)
			Rep[i]=Rep[i]+M[i][j]*A[j];
	}

	for (i=0; i<4; i++)
		R[i]=Rep[i];
}


/**
* Fonction multipliant une matrice 3x3 
* par une matrice 3x3
*/
void MulMatMat3(TypeMat3 M1, TypeMat3 M2, TypeMat3 R)
{
	int		i, j, k;
	TypeMat3	Rep;
	
	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
		{
			Rep[i][j]=0;
			for (k=0; k<3; k++)
				Rep[i][j]=Rep[i][j]+M1[i][k]*M2[k][j];
		}

	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			R[i][j]=Rep[i][j];
}


/**
* Fonction multipliant une matrice 
* homogène 4x4 par une matrice homogène 4x4
*/
void MulMatMatH4(TypeMatH4 M1, TypeMatH4 M2, TypeMatH4 R)
{
	int		i, j, k;
	TypeMatH4	Rep;
	
	for (i=0; i<4; i++)
		for (j=0; j<4; j++)
		{
			Rep[i][j]=0;
			for (k=0; k<4; k++)
				Rep[i][j]=Rep[i][j]+M1[i][k]*M2[k][j];
		}

	for (i=0; i<4; i++)
		for (j=0; j<4; j++)
			R[i][j]=Rep[i][j];
}


/**
* Fonction multipliant une matrice 6x6 
* par une matrice 6x6
*/
void MulMatMat6(TypeMat6 M1, TypeMat6 M2, TypeMat6 R)
{
	int		i, j, k;
	TypeMat6	Rep;
	
	for (i=0; i<6; i++)
		for (j=0; j<6; j++)
		{
			Rep[i][j]=0;
			for (k=0; k<6; k++)
				Rep[i][j]=Rep[i][j]+M1[i][k]*M2[k][j];
		}

	for (i=0; i<6; i++)
		for (j=0; j<6; j++)
			R[i][j]=Rep[i][j];
}


/**
* Fonction sommant deux vecteurs 3x1 
*/
void SomVectVect3(TypeVect3 A,TypeVect3 B, TypeVect3 R)
{
	int i;

	for (i=0; i<3; i++)
		R[i]=A[i]+B[i];
}


/**
* Fonction soustrayant deux vecteurs 3x1 
*/
void SubVectVect3(TypeVect3 A, TypeVect3 B, TypeVect3 R)
{
	int i;

	for (i=0; i<3; i++)
		R[i]=A[i]-B[i];
}


/**
* Fonction sommant deux vecteurs 
* homogènes 4x1 
*/
void SomVectVectH4(TypeVectH4 A, TypeVectH4 B, TypeVectH4 R)
{
	int i;

	for (i=0; i<3; i++)
		R[i]=A[i]+B[i];
	R[3]=1;
}


/**
* Fonction soustrayant deux vecteurs 
* homogènes 4x1 
*/
void SubVectVectH4(TypeVectH4 A, TypeVectH4 B, TypeVectH4 R)
{
	int i;

	for (i=0; i<3; i++)
		R[i]=A[i]-B[i];
	R[3]=1;
}


/**
* Fonction sommant deux vecteurs 6x1 
*/
void SomVectVect6(TypeVect6 A, TypeVect6 B, TypeVect6 R)
{
	int i;

	for (i=0; i<6; i++)
		R[i]=A[i]+B[i];
}


/**
* Fonction soustrayant deux vecteurs 6x1 
*/
void SubVectVect6(TypeVect6 A, TypeVect6 B, TypeVect6 R)
{
	int i;

	for (i=0; i<6; i++)
		R[i]=A[i]-B[i];
}


/**
* Fonction sommant deux matrices 3x3 
*/
void SomMatMat3(TypeMat3 A, TypeMat3 B, TypeMat3 R)
{
	int i, j;

	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			R[i][j]=A[i][j]+B[i][j];
}


/**
* Fonction soustrayant deux matrices 3x3 
*/
void SubMatMat3(TypeMat3 A, TypeMat3 B, TypeMat3 R)
{
	int i, j;

	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			R[i][j]=A[i][j]-B[i][j];
}


/**
* Fonction sommant deux matrices 6x6 
*/
void SomMatMat6(TypeMat6 A, TypeMat6 B, TypeMat6 R)
{
	int i, j;

	for (i=0; i<6; i++)
		for (j=0; j<6; j++)
			R[i][j]=A[i][j]+B[i][j];
}


/**
* Fonction soustrayant deux matrices 6x6 
*/
void SubMatMat6(TypeMat6 A, TypeMat6 B, TypeMat6 R)
{
	int i, j;

	for (i=0; i<6; i++)
		for (j=0; j<6; j++)
			R[i][j]=A[i][j]-B[i][j];
}


/**
* Fonction initialisant un vecteur 3x1 
*/
void InitialVect3(TypeVect3 A)
{
	A[0]=0.0f;
	A[1]=0.0f;
	A[2]=0.0f;
}


/**
* Fonction initialisant un vecteur 
* homogène 4x1 
*/
void InitialVectH4(TypeVectH4 A)
{
	A[0]=0.0f;
	A[1]=0.0f;
	A[2]=0.0f;
	A[3]=1.0f;
}


/**
* Fonction initialisant un vecteur 6x1 
*/
void InitialVect6(TypeVect6 A)
{
	A[0]=0.0f;
	A[1]=0.0f;
	A[2]=0.0f;
	A[3]=0.0f;
	A[4]=0.0f;
	A[5]=0.0f;
}


/**
* Fonction initialisant une matrice 3x3 
*/
void InitialMat3(TypeMat3 A)
{
	int i, j;
	
	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			A[i][j]=0.0f;
}


/**
* Fonction initialisant une matrice 6x6
*/
void InitialMat6(TypeMat6 A)
{
	int i, j;
	
	for (i=0; i<6; i++)
		for (j=0; j<6; j++)
			A[i][j]=0.0f;
}


/**
* Fonction créant une matrice identité 3x3 
*/
void IdMat3(TypeMat3 A)
{
	int i, j;
	
	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			A[i][j]=0.0f;
	A[0][0]=1.0f;
	A[1][1]=1.0f;
	A[2][2]=1.0f;
}


/**
* Fonction créant une matrice 
* homogène identité 4x4
*/
void IdMatH4(TypeMatH4 A)
{
	int i, j;
	
	for (i=0; i<4; i++)
		for (j=0; j<4; j++)
			A[i][j]=0.0f;
	A[0][0]=1.0f;
	A[1][1]=1.0f;
	A[2][2]=1.0f;
	A[3][3]=1.0f;
}


/**
* Fonction créant une matrice identité 6x6 
*/
void IdMat6(TypeMat6 A)
{
	int i, j;
	
	for (i=0; i<6; i++)
		for (j=0; j<6; j++)
			A[i][j]=0.0f;
	A[0][0]=1.0f;
	A[1][1]=1.0f;
	A[2][2]=1.0f;
	A[3][3]=1.0f;
	A[4][4]=1.0f;
	A[5][5]=1.0f;
}


/**
* Fonction calculant la norme au carré 
* d'un vecteur 3x1
*/
double NormeCarreVect3(TypeVect3 A)
{
	int		i;
	double	norme=0.0f;

	for (i=0; i<3; i++)
		norme=norme+A[i]*A[i];
	return norme;
}


/**
* Fonction calculant la norme au carré 
* d'un vecteur homogène 4x1
*/
double NormeCarreVectH4(TypeVectH4 A)
{
	int		i;
	double	norme=0.0f;

	for (i=0; i<3; i++)
		norme=norme+A[i]*A[i];
	return norme;
}


/**
* Fonction créant un vecteur homogène 4x1  
* à partir d'un vecteur position 3x1
*/
void MakeVectH4Pos(TypeVect3 Pos, TypeVectH4 V)
{
	V[0]=Pos[0];
	V[1]=Pos[1];
	V[2]=Pos[2];
	V[3]=1;
}


/**
* Fonction créant une matrice de rotation 3x3  
* à partir d'un vecteur de position 
* angulaire 3x1
*/
void MakeMat3Rot(TypeVect3 Ang, TypeMat3 R)
{
	double sinalpha=sin(Ang[0]);
	double cosalpha=cos(Ang[0]);
	double sinbeta=sin(Ang[1]);
	double cosbeta=cos(Ang[1]);
	double singamma=sin(Ang[2]);
	double cosgamma=cos(Ang[2]);
	
	R[0][0]=cosbeta*cosgamma;
	R[0][1]=cosgamma*sinalpha*sinbeta - cosalpha*singamma;
	R[0][2]=cosalpha*cosgamma*sinbeta + sinalpha*singamma;
	R[1][0]=cosbeta*singamma;
	R[1][1]=cosalpha*cosgamma + sinalpha*sinbeta*singamma;
	R[1][2]=-cosgamma*sinalpha + cosalpha*sinbeta*singamma;
	R[2][0]=-sinbeta;
	R[2][1]=cosbeta*sinalpha;
	R[2][2]=cosalpha*cosbeta;
}


/**
* Fonction créant une matrice homogène 4x4  
* à partir d'un vecteur position angulaire 
* 3x1 et d'un vecteur de position 3x1
*/
void MakeMatH4AngPos(TypeVect3 Ang, TypeVect3 Pos, TypeMatH4 H)
{
	double sinalpha=sin(Ang[0]);
	double cosalpha=cos(Ang[0]);
	double sinbeta=sin(Ang[1]);
	double cosbeta=cos(Ang[1]);
	double singamma=sin(Ang[2]);
	double cosgamma=cos(Ang[2]);
	
	H[0][0]=cosbeta*cosgamma;
	H[0][1]=cosgamma*sinalpha*sinbeta - cosalpha*singamma;
	H[0][2]=cosalpha*cosgamma*sinbeta + sinalpha*singamma;
	H[0][3]=Pos[0];
	H[1][0]=cosbeta*singamma;
	H[1][1]=cosalpha*cosgamma + sinalpha*sinbeta*singamma;
	H[1][2]=-cosgamma*sinalpha + cosalpha*sinbeta*singamma;
	H[1][3]=Pos[1];
	H[2][0]=-sinbeta;
	H[2][1]=cosbeta*sinalpha;
	H[2][2]=cosalpha*cosbeta;
	H[2][3]=Pos[2];
	H[3][0]=0.0f;
	H[3][1]=0.0f;
	H[3][2]=0.0f;
	H[3][3]=1.0f;
}


/**
* Fonction créant une matrice homogène 4x4  
* à partir d'une matrice de rotation 3x3 
* et d'un vecteur de position 3x1
*/
void MakeMatH4RotPos(TypeMat3 R, TypeVect3 Pos, TypeMatH4 H)
{
	H[0][0]=R[0][0];
	H[0][1]=R[0][1];
	H[0][2]=R[0][2];
	H[0][3]=Pos[0];
	H[1][0]=R[1][0];
	H[1][1]=R[1][1];
	H[1][2]=R[1][2];
	H[1][3]=Pos[1];
	H[2][0]=R[2][0];
	H[2][1]=R[2][1];
	H[2][2]=R[2][2];
	H[2][3]=Pos[2];
	H[3][0]=0.0f;
	H[3][1]=0.0f;
	H[3][2]=0.0f;
	H[3][3]=1.0f;
}


/**
* Fonction créant une matrice homogène 4x4  
* à partir d'un vecteur de position angulaire 
* inverse 3x1 
* et d'un vecteur de position 3x1
*/
void MakeMatH4InvAngPos(TypeVect3 Ang, TypeVect3 Pos, TypeMatH4 H)
{
	double sinalpha=sin(Ang[0]);
	double cosalpha=cos(Ang[0]);
	double sinbeta=sin(Ang[1]);
	double cosbeta=cos(Ang[1]);
	double singamma=sin(Ang[2]);
	double cosgamma=cos(Ang[2]);
	
	H[0][0]=cosbeta*cosgamma;
	H[0][1]=cosbeta*singamma;
	H[0][2]=-sinbeta;
	H[0][3]=-Pos[0]*cosbeta*cosgamma + Pos[2]*sinbeta - Pos[1]*cosbeta*singamma;
	H[1][0]=cosgamma*sinalpha*sinbeta - cosalpha*singamma;
	H[1][1]=cosalpha*cosgamma + sinalpha*sinbeta*singamma;
	H[1][2]=cosbeta*sinalpha;
	H[1][3]=-Pos[2]*cosbeta*sinalpha - Pos[0]*(cosgamma*sinalpha*sinbeta - cosalpha*singamma)
		- Pos[1]*(cosalpha*cosgamma + sinalpha*sinbeta*singamma);
	H[2][0]=cosalpha*cosgamma*sinbeta + sinalpha*singamma;
	H[2][1]=-cosgamma*sinalpha + cosalpha*sinbeta*singamma;
	H[2][2]=cosalpha*cosbeta;
	H[2][3]=-Pos[2]*cosalpha*cosbeta - Pos[0]*(cosalpha*cosgamma*sinbeta + sinalpha*singamma)
		- Pos[1]*(-cosgamma*sinalpha + cosalpha*sinbeta*singamma);
	H[3][0]=0.0f;
	H[3][1]=0.0f;
	H[3][2]=0.0f;
	H[3][3]=1.0f;
}


/**
* Fonction créant une matrice homogène 4x4  
* à partir d'une matrice de rotation inverse
* 3x3 et d'un vecteur de position 3x1
*/
void MakeMatH4InvRotPos(TypeMat3 R, TypeVect3 Pos, TypeMatH4 H)
{
	H[0][0]=R[0][0];
	H[0][1]=R[1][0];
	H[0][2]=R[2][0];
	H[0][3]=-Pos[0]*R[0][0] - Pos[1]*R[1][0] - Pos[2]*R[2][0];
	H[1][0]=R[0][1];
	H[1][1]=R[1][1];
	H[1][2]=R[2][1];
	H[1][3]=-Pos[0]*R[0][1] - Pos[1]*R[1][1] - Pos[2]*R[2][1] ;
	H[2][0]=R[0][2];
	H[2][1]=R[1][2];
	H[2][2]=R[2][2];
	H[2][3]=-Pos[0]*R[0][2] - Pos[1]*R[1][2] - Pos[2]*R[2][2] ;
	H[3][0]=0.0f;
	H[3][1]=0.0f;
	H[3][2]=0.0f;
	H[3][3]=1.0f;
}


/**
* Fonction créant une matrice homogène 4x4  
* à partir d'un vecteur de 
* position généralisée 6x1
*/
void MakeMatH4PosGen(TypeVect6 Pos,TypeMatH4 H)
{
	double sinalpha=sin(Pos[alpha]);
	double cosalpha=cos(Pos[alpha]);
	double sinbeta=sin(Pos[beta]);
	double cosbeta=cos(Pos[beta]);
	double singamma=sin(Pos[gamma]);
	double cosgamma=cos(Pos[gamma]);
	
	H[0][0]=cosbeta*cosgamma;
	H[0][1]=cosgamma*sinalpha*sinbeta - cosalpha*singamma;
	H[0][2]=cosalpha*cosgamma*sinbeta + sinalpha*singamma;
	H[0][3]=Pos[0];
	H[1][0]=cosbeta*singamma;
	H[1][1]=cosalpha*cosgamma + sinalpha*sinbeta*singamma;
	H[1][2]=-cosgamma*sinalpha + cosalpha*sinbeta*singamma;
	H[1][3]=Pos[1];
	H[2][0]=-sinbeta;
	H[2][1]=cosbeta*sinalpha;
	H[2][2]=cosalpha*cosbeta;
	H[2][3]=Pos[2];
	H[3][0]=0.0f;
	H[3][1]=0.0f;
	H[3][2]=0.0f;
	H[3][3]=1.0f;
}


/**
* Fonction créant une matrice 6x6 liant 
* la vitesse d'un point C attaché au point B 
* à la vitesse du point B, 
* le tout exprimé dans un référentiel A
*/
void MakeMat6RotPVit(TypeMat3 R,TypeVect6 Pos, TypeMat6 O)
{
	O[0][0] = 1.0f;
	O[0][1] = 0.0f;
	O[0][2] = 0.0f;
	O[0][3] = 0.0f;
	O[0][4] =  R[2][0]*Pos[0] + R[2][1]*Pos[1] + R[2][2]*Pos[2];
	O[0][5] = -R[1][0]*Pos[0] - R[1][1]*Pos[1] - R[1][2]*Pos[2];
	O[1][0] = 0.0f;
	O[1][1] = 1.0f;
	O[1][2] = 0.0f;
	O[1][3] = -R[2][0]*Pos[0] - R[2][1]*Pos[1] - R[2][2]*Pos[2];
	O[1][4] = 0.0f;
	O[1][5] =  R[0][0]*Pos[0] + R[0][1]*Pos[1] + R[0][2]*Pos[2];
	O[2][0] = 0.0f;
	O[2][1] = 0.0f;
	O[2][2] = 1.0f;
	O[2][3] =  R[1][0]*Pos[0] + R[1][1]*Pos[1] + R[1][2]*Pos[2];
	O[2][4] = -R[0][0]*Pos[0] - R[0][1]*Pos[1] - R[0][2]*Pos[2];
	O[2][5] = 0.0f;
	O[3][0] = 0.0f;
	O[3][1] = 0.0f;
	O[3][2] = 0.0f;
	O[3][3] = 1.0f;
	O[3][4] = 0.0f;
	O[3][5] = 0.0f;
	O[4][0] = 0.0f;
	O[4][1] = 0.0f;
	O[4][2] = 0.0f;
	O[4][3] = 0.0f;
	O[4][4] = 1.0f;
	O[4][5] = 0.0f;
	O[5][0] = 0.0f;
	O[5][1] = 0.0f;
	O[5][2] = 0.0f;
	O[5][3] = 0.0f;
	O[5][4] = 0.0f;
	O[5][5] = 1.0f;
}


/**
* Fonction créant une matrice liant 
* la force en un point C à la force 
* équivalente en un point B, 
* le tout exprimé dans un référentiel A
*/
void MakeMat6RotPForce(TypeMat3 R,TypeVect6 Pos, TypeMat6 O)
{
	O[0][0] = 1.0f;
	O[0][1] = 0.0f;
	O[0][2] = 0.0f;
	O[0][3] = 0.0f;
	O[0][4] = 0.0f;
	O[0][5] = 0.0f;
	O[1][0] = 0.0f;
	O[1][1] = 1.0f;
	O[1][2] = 0.0f;
	O[1][3] = 0.0f;
	O[1][4] = 0.0f;
	O[1][5] = 0.0f;
	O[2][0] = 0.0f;
	O[2][1] = 0.0f;
	O[2][2] = 1.0f;
	O[2][3] = 0.0f;
	O[2][4] = 0.0f;
	O[2][5] = 0.0f;
	O[3][0] = 0.0f;
	O[3][1] = -R[2][0]*Pos[0] - R[2][1]*Pos[1] - R[2][2]*Pos[2];
	O[3][2] =  R[1][0]*Pos[0] + R[1][1]*Pos[1] + R[1][2]*Pos[2];
	O[3][3] = 1.0f;
	O[3][4] = 0.0f;
	O[3][5] = 0.0f;
	O[4][0] =  R[2][0]*Pos[0] + R[2][1]*Pos[1] + R[2][2]*Pos[2];
	O[4][1] = 0.0f;
	O[4][2] = -R[0][0]*Pos[0] - R[0][1]*Pos[1] - R[0][2]*Pos[2];
	O[4][3] = 0.0f;
	O[4][4] = 1.0f;
	O[4][5] = 0.0f;
	O[5][0] = -R[1][0]*Pos[0] - R[1][1]*Pos[1] - R[1][2]*Pos[2];
	O[5][1] =  R[0][0]*Pos[0] + R[0][1]*Pos[1] + R[0][2]*Pos[2];
	O[5][2] = 0.0f;
	O[5][3] = 0.0f;
	O[5][4] = 0.0f;
	O[5][5] = 1.0f;
}


/**
* Fonction créant une matrice 6x6 contenant
* deux matrices de rotation 3x3 
* sur la diagonale 
*/
void MakeMat6Rot(TypeMat3 R,TypeMat6 O)
{
	O[0][0] = R[0][0];
	O[0][1] = R[0][1];
	O[0][2] = R[0][2];
	O[0][3] = 0.0f;
	O[0][4] = 0.0f;
	O[0][5] = 0.0f;
	O[1][0] = R[1][0];
	O[1][1] = R[1][1];
	O[1][2] = R[1][2];
	O[1][3] = 0.0f;
	O[1][4] = 0.0f;
	O[1][5] = 0.0f;
	O[2][0] = R[2][0];
	O[2][1] = R[2][1];
	O[2][2] = R[2][2];
	O[2][3] = 0.0f;
	O[2][4] = 0.0f;
	O[2][5] = 0.0f;
	O[3][0] = 0.0f;
	O[3][1] = 0.0f;
	O[3][2] = 0.0f;
	O[3][3] = R[0][0];
	O[3][4] = R[0][1];
	O[3][5] = R[0][2];
	O[4][0] = 0.0f;
	O[4][1] = 0.0f;
	O[4][2] = 0.0f;
	O[4][3] = R[1][0];
	O[4][4] = R[1][1];
	O[4][5] = R[1][2];
	O[5][0] = 0.0f;
	O[5][1] = 0.0f;
	O[5][2] = 0.0f;
	O[5][3] = R[2][0];
	O[5][4] = R[2][1];
	O[5][5] = R[2][2];
}


/**
* Fonction créant une matrice de gain 6x6 
* ayant comme diagonale les valeurs 
* fournies en entrée  
*/
void MakeMat6Gain(double gain_1,double gain_2,double gain_3,double gain_4,double gain_5,double gain_6,TypeMat6 R)
{
	R[0][0] = gain_1;
	R[0][1] = 0.0f;
	R[0][2] = 0.0f;
	R[0][3] = 0.0f;
	R[0][4] = 0.0f;
	R[0][5] = 0.0f;
	R[1][0] = 0.0f;
	R[1][1] = gain_2;
	R[1][2] = 0.0f;
	R[1][3] = 0.0f;
	R[1][4] = 0.0f;
	R[1][5] = 0.0f;
	R[2][0] = 0.0f;
	R[2][1] = 0.0f;
	R[2][2] = gain_3;
	R[2][3] = 0.0f;
	R[2][4] = 0.0f;
	R[2][5] = 0.0f;
	R[3][0] = 0.0f;
	R[3][1] = 0.0f;
	R[3][2] = 0.0f;
	R[3][3] = gain_4;
	R[3][4] = 0.0f;
	R[3][5] = 0.0f;
	R[4][0] = 0.0f;
	R[4][1] = 0.0f;
	R[4][2] = 0.0f;
	R[4][3] = 0.0f;
	R[4][4] = gain_5;
	R[4][5] = 0.0f;
	R[5][0] = 0.0f;
	R[5][1] = 0.0f;
	R[5][2] = 0.0f;
	R[5][3] = 0.0f;
	R[5][4] = 0.0f;
	R[5][5] = gain_6;
}


/**
* Fonction créant un vecteur de gain 6x1 
* à partir des valeurs fournies en entrée
*/
void MakeVect6Gain(double gain_1,double gain_2,double gain_3,double gain_4,double gain_5,double gain_6,TypeVect6 R)
{
	R[0]= gain_1;
	R[1]= gain_2;
	R[2]= gain_3;
	R[3]= gain_4;
	R[4]= gain_5;
	R[5]= gain_6;
}
 

/**
* Fonction extrayant la position 
* angulaire dans un vecteur 3x1
* à partir d'une matrice de rotation 3x3
*/
void ExtractAngMat3(TypeMat3 R, TypeVect3 Ang)
// ne fonctionne correctement que pour -90°<beta<90°, cos^2(beta) seul accessible
// si cos(beta) prévu < 0, alors ajouter signe "-" devant sqrt
{
	double cosbeta;

	Ang[1]=atan2(-R[2][0], sqrt(R[0][0]*R[0][0]+R[1][0]*R[1][0]));
	cosbeta=cos(Ang[1]);
	if (fabs(cosbeta)>=EPSILON_DIV_ZERO_DOUBLE)
	{
		Ang[0]=atan2(R[2][1]/cosbeta, R[2][2]/cosbeta);
		Ang[2]=atan2(R[1][0]/cosbeta, R[0][0]/cosbeta);
	}
	else
	{
		if (R[2][0]<=0)
		{
			Ang[2]=0.0;
			Ang[0]=atan2(R[0][1], R[1][1]);
		}
		else
		{
			Ang[2]=0.0;
			Ang[0]=-atan2(R[0][1], R[1][1]);
		}
	}
}


/**
* Fonction extrayant la position 
* angulaire dans un vecteur 3x1
* à partir d'une matrice homogène 4x4
*/
void ExtractAngMatH4(TypeMatH4 H, TypeVect3 Ang)
// ne fonctionne correctement que pour -90°<beta<90°, cos^2(beta) seul accessible
// si cos(beta) prévu < 0, alors ajouter signe "-" devant sqrt
{
	double cosbeta;

	Ang[1]=atan2(-H[2][0], sqrt(H[0][0]*H[0][0]+H[1][0]*H[1][0]));
	cosbeta=cos(Ang[1]);
	if (fabs(cosbeta)>=EPSILON_DIV_ZERO_DOUBLE)
	{
		Ang[0]=atan2(H[2][1]/cosbeta, H[2][2]/cosbeta);
		Ang[2]=atan2(H[1][0]/cosbeta, H[0][0]/cosbeta);
	}
	else
	{
		if (H[2][0]<=0)
		{
			Ang[2]=0.0;
			Ang[0]=atan2(H[0][1], H[1][1]);
		}
		else
		{
			Ang[2]=0.0;
			Ang[0]=-atan2(H[0][1], H[1][1]);
		}
	}
}


/**
* Fonction extrayant la position 
* dans un vecteur 3x1
* à partir d'un vecteur homogène 4x1
*/
void ExtractPosVectH4(TypeVectH4 V, TypeVect3 Pos)
{
	Pos[axeX]=V[0];
	Pos[axeY]=V[1];
	Pos[axeZ]=V[2];
}


/**
* Fonction extrayant la position 
* dans un vecteur 3x1
* à partir d'une matrice homogène 4x4
*/
void ExtractPosMatH4(TypeMatH4 H, TypeVect3 Pos)
{
	Pos[axeX]=H[0][3];
	Pos[axeY]=H[1][3];
	Pos[axeZ]=H[2][3];
}


/**
* Fonction extrayant la matrice 
* de rotation 3x3
* à partir d'une matrice homogène 4x4
*/
void ExtractRotMatH4(TypeMatH4 H, TypeMat3 R)
{
	R[0][0]=H[0][0];
	R[0][1]=H[0][1];
	R[0][2]=H[0][2];
	R[1][0]=H[1][0];
	R[1][1]=H[1][1];
	R[1][2]=H[1][2];
	R[2][0]=H[2][0];
	R[2][1]=H[2][1];
	R[2][2]=H[2][2];
}


/**
* Fonction extrayant la position 
* généralisée dans un vecteur 6x1
* à partir d'une matrice homogène 4x4
*/
void ExtractPosGenMatH4(TypeMatH4 H,TypeVect6 Pos)
// ne fonctionne correctement que pour -90°<beta<90°, cos^2(beta) seul accessible
// si cos(beta) prévu < 0, alors ajouter signe "-" devant sqrt
{
	double cosbeta;

	Pos[beta]=atan2(-H[2][0], sqrt(H[0][0]*H[0][0]+H[1][0]*H[1][0]));
	cosbeta=cos(Pos[beta]);
	if (fabs(cosbeta)>=EPSILON_DIV_ZERO_DOUBLE)
	{
		Pos[alpha]=atan2(H[2][1]/cosbeta, H[2][2]/cosbeta);
		Pos[gamma]=atan2(H[1][0]/cosbeta, H[0][0]/cosbeta);
	}
	else
	{
		if (H[2][0]<=0)
		{
			Pos[gamma]=0.0;
			Pos[alpha]=atan2(H[0][1], H[1][1]);
		}
		else
		{
			Pos[gamma]=0.0;
			Pos[alpha]=-atan2(H[0][1], H[1][1]);
		}
	}
	Pos[axeX]=H[0][3];
	Pos[axeY]=H[1][3];
	Pos[axeZ]=H[2][3];
}


/**
* Fonction calculant l'inverse d'une matrice
* 6x6. Le résultat est dans la matrice A.
*/
void InvMat6(TypeMat6 A, TypeMat6 B)
// prend 46us sur pentium 500MHz n=6, m=1
{
	const	int n=6, m=1; // n=rang de A et m=nb de vect col de B à évaluer
	int		i, icol=0, irow=0, j, k, l, ll;
	double	big, dum, pivinv, temp;
	int		Indxc[n], Indxr[n], Ipiv[n];


	for (j=0; j<n; j++)
		Ipiv[j]=0;
	for (i=0; i<n; i++) // boucle principale sur les col à réduire
	{
		big=0.0;
		for (j=0; j<n; j++) // boucle extérieure de recherche d'un élément pivot
			if (Ipiv[j]!=1)
				for (k=0; k<n; k++)
				{
					if (Ipiv[k]==0)
					{
						if (fabs(A[j][k])>=big)
						{
							big=fabs(A[j][k]);
							irow=j;
							icol=k;
						}
					}
					else
						if (Ipiv[k]>1)
							printf("Erreur d'inversion pt 1: matrice singulière");
				}
		++(Ipiv[icol]);

		if (irow != icol)
		{
			for (l=0; l<n; l++)
			{
				temp=A[irow][l];
				A[irow][l]=A[icol][l];
				A[icol][l]=temp;
			}
			for (l=0; l<m; l++)
			{
				temp=B[irow][l];
				B[irow][l]=B[icol][l];
				B[icol][l]=temp;
			}
		}

		Indxr[i]=irow;
		Indxc[i]=icol;
		if (fabs(A[icol][icol])<=EPSILON_DIV_ZERO_DOUBLE)
			printf("Erreur d'inversion pt 2: matrice singulière");
		pivinv=1.0/A[icol][icol];
		A[icol][icol]=1.0;
		for (l=0; l<n; l++)
			A[icol][l]*=pivinv;
		for (l=0; l<m; l++)
			B[icol][l]*=pivinv;

		for (ll=0; ll<n; ll++)
		{
			if (ll != icol)
			{
				dum=A[ll][icol];
				A[ll][icol]=0.0;
				for (l=0; l<n; l++)
					A[ll][l]-=A[icol][l]*dum;
				for (l=0; l<m; l++)
					B[ll][l]-=B[icol][l]*dum;
			}
		}

	}
	for (l=n-1; l>=0; l--)
	{
		if (Indxr[l] != Indxc[l])
			for (k=0; k<n; k++)
			{
				temp=A[k][Indxr[l]];
				A[k][Indxr[l]]=A[k][Indxc[l]];
				A[k][Indxc[l]]=temp;
			}
	}
} 



////////////////////////////////////////////////////////////////////////////////
// File: doolittle_pivot.c                                                    //
// Routines:                                                                  //
//    Doolittle_LU_Decomposition_with_Pivoting                                //
//    Doolittle_LU_with_Pivoting_Solve                                        //
////////////////////////////////////////////////////////////////////////////////
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
//                                                                            //

//#include <math.h>                                     // required for fabs()

int Doolittle_LU_Decomposition_with_Pivoting(double *A, int pivot[], int n)
{
   int i, j, k;
   double *p_k = NULL, *p_row = NULL, *p_col = NULL;
   double max;


//         For each row and column, k = 0, ..., n-1,
 
   for (k = 0, p_k = A; k < n; p_k += n, k++) {

//            find the pivot row

      pivot[k] = k;
      max = fabs( *(p_k + k) );
      for (j = k + 1, p_row = p_k + n; j < n; j++, p_row += n) {
         if ( max < fabs(*(p_row + k)) ) {
            max = fabs(*(p_row + k));
            pivot[k] = j;
            p_col = p_row;
         }
      }

//     and if the pivot row differs from the current row, then
//     interchange the two rows.
   
      if (pivot[k] != k)
         for (j = 0; j < n; j++) {
            max = *(p_k + j);
            *(p_k + j) = *(p_col + j);
            *(p_col + j) = max;
         }

//                and if the matrix is singular, return error


      if ( *(p_k + k) == 0.0 ) return -1;

//      otherwise find the lower triangular matrix elements for column k. 

      for (i = k+1, p_row = p_k + n; i < n; p_row += n, i++) {
         *(p_row + k) /= *(p_k + k);
      }  

//            update remaining matrix

      for (i = k+1, p_row = p_k + n; i < n; p_row += n, i++)
         for (j = k+1; j < n; j++)
            *(p_row + j) -= *(p_row + k) * *(p_k + j);

   }

   return 0;
}



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
//                                                                            //
int Doolittle_LU_with_Pivoting_Solve(double *A, double B[], int pivot[],
                                                              double x[], int n)
{
   int i, k;
   double *p_k;
   double dum;

//         Solve the linear equation Lx = B for x, where L is a lower
//         triangular matrix with an implied 1 along the diagonal.
   
   for (k = 0, p_k = A; k < n; p_k += n, k++) {
      if (pivot[k] != k) {dum = B[k]; B[k] = B[pivot[k]]; B[pivot[k]] = dum; }
      x[k] = B[k];
      for (i = 0; i < k; i++) x[k] -= x[i] * *(p_k + i);
   }

//         Solve the linear equation Ux = y, where y is the solution
//         obtained above of Lx = B and U is an upper triangular matrix.

   for (k = n-1, p_k = A + n*(n-1); k >= 0; k--, p_k -= n) {
      if (pivot[k] != k) {dum = B[k]; B[k] = B[pivot[k]]; B[pivot[k]] = dum; }
      for (i = k + 1; i < n; i++) x[k] -= x[i] * *(p_k + i);
      if (*(p_k + k) == 0.0) return -1;
      x[k] /= *(p_k + k);
   }
  
   return 0;
}



/**
* Fonction calculant la norme au carré 
* d'un vecteur nx1
*/
double NormeCarreVect(double* V, int n)
{
	int		i;
	double	norme=0.0f;

	for (i=0; i<n; i++)
		norme=norme+V[i]*V[i];
	return norme;
}
