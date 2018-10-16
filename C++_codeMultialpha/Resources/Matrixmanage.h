#ifndef MATRIXMANAGE_H
#define MATRIXMANAGE_H




///------------------------------------------------------------------------------
///---------------------- conventional application with matrices ----------------
///------------------------------------------------------------------------------
#include <opencv2/opencv.hpp>

using namespace cv;

///------------------------------------------------------------------------------
///---------------------- 4D Matrices  ------------------------------------------
///------------------------------------------------------------------------------
float**** ToCreateMatrix4D(int n, int m, int l, int p);
void ToInitMatrix4D(float**** M4D, int n, int m, int l, int p);
void ToEliminateMatrix4D(float**** M4D, int n, int m, int l, int p);
void ToPrintMatrix4D(float**** M4D, int n, int m, int l, int p);

///------------------------------------------------------------------------------
///---------------------- 3D Matrices  ------------------------------------------
///------------------------------------------------------------------------------

void ToInitMatrix3D(float*** M3D, int n, int m, int l);
void ToInitWithLabelMatrix3D(float*** M3D, int n, int m, int l, float label);
float*** ToCreateMatrix3D(int n, int m, int l);
void ToEliminateMatrix3D(float*** M3D, int n, int m, int l);
void ToPrintMatrix3D(float*** M3D, int n, int m, int l);

///------------------------------------------------------------------------------
///---------------------- 2D Matrices  ------------------------------------------
///------------------------------------------------------------------------------

void ToInitMatrix2D(float** M2D, int n, int m);
float** ToCreateMatrix2D(int n, int m);
void ToEliminateMatrix2D(float** M2D, int n, int m);
void ToPrintMatrix2D(float** M2D, int n, int m);
void ToInitWithLabelMatrix2D(float** M2D, int n, int m, float label);

///------------------------------------------------------------------------------
///---------------------- 1D Matrices = vectors ---------------------------------
///------------------------------------------------------------------------------

void ToInitMatrix1D(float* M1D, int n);
float* ToCreateMatrix1D(int n);
void ToEliminateMatrix1D(float* M1D, int n);
void ToPrintMatrix1D(float* M1D, int n);


///------------------------------------------------------------------------------
///---------------------- Mat Matrices  -----------------------------------------
///------------------------------------------------------------------------------


Mat** ToCreateMatrixMat2D(int n, int m);
void ToEliminateMatrixMat2D(Mat** M2D, int n, int m);

///------------------------------------------------------------------------------

#endif // MATRIXMANAGE_H
