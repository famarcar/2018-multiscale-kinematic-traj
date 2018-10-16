///------------------------------------------------------------------------------
///---------------------- conventional application with matrices ----------------
///------------------------------------------------------------------------------

#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace std;
using namespace cv;



///-----------------------------------------------------------------------------
///------------------- 4D Matrices ---------------------------------------------
///------------------------------------------------------------------------------

void ToPrintMatrix4D(float**** M4D, int n, int m, int l, int p)
{
    for(int i=0; i<n; i++)
    {
        for(int j=0; j<m; j++)
        {
            for(int k=0; k<l; k++)
            {
                for(int q=0; q<p; q++)
                    cout<< M4D[i][j][k][q]<< " ";
            }
            cout<<"     -       ";
        }
        cout<<endl;
    }
}


void ToInitMatrix4D(float**** M4D, int n, int m, int l, int p)
{

    for(int i=0; i<n; i++)
    {
        for(int j=0; j<m; j++)
        {
            for(int k=0; k<l; k++)
            {
                for(int z=0; z<p; z++)
                {
                    M4D[i][j][k][z]=0.0f;
                }
            }
        }
    }
}






float**** ToCreateMatrix4D(int n, int m, int l, int p)
{
    float **** X = new float***[n];
    for(int i=0; i<n; i++)
    {
        X[i] = new float**[m];
        for(int j=0; j< m; j++)
        {
            X[i][j] = new float*[l];
            for(int k=0; k<l; k++)
            {
                X[i][j][k]= new float[p];
            }

        }
    }
    return X;
}





void ToEliminateMatrix4D(float**** M4D, int n, int m, int l, int p)
{

    for(int i=0; i<n; i++)
    {
        for(int j=0; j< m; j++)
        {
            for(int k=0; k<l; k++)
            {

                delete [] M4D[i][j][k];
            }
            delete [] M4D[i][j];
        }
        delete[] M4D[i];
    }
    delete M4D;

    return;
}



///------------------------------------------------------------------------------
///---------------------- 3D Matrices  ------------------------------------------
///------------------------------------------------------------------------------

void ToPrintMatrix3D(float*** M3D, int n, int m, int l)
{
    for(int i=0; i<n; i++)
    {
        for(int j=0; j<m; j++)
        {
            for(int k=0; k<l; k++)
            {
                cout<< M3D[i][j][k]<< " ";
            }
            cout<<"     -       ";
        }
        cout<<endl;
    }
}


void ToInitMatrix3D(float*** M3D, int n, int m, int l)
{

    for(int i=0; i<n; i++)
    {
        for(int j=0; j<m; j++)
        {
            for(int k=0; k<l; k++)
            {
                M3D[i][j][k]=0.0f;
            }
        }
    }
}


float*** ToCreateMatrix3D(int n, int m, int l)
{
    float *** X = new float**[n];
    for(int i=0; i<n; i++)
    {
        X[i] = new float*[m];
        for(int j=0; j< m; j++)
        {
            X[i][j] = new float[l];
        }
    }
    return X;
}


void ToInitWithLabelMatrix3D(float*** M3D, int n, int m, int l, float label)
{
    for(int i=0; i<n; i++)
    {
        for(int j=0; j<m; j++)
        {
            for(int k=0; k<l; k++)
            {
                M3D[i][j][k]=label;
            }
        }
    }
}


void ToEliminateMatrix3D(float*** M3D, int n, int m, int l)
{

    for(int i=0; i<n; i++)
    {
        for(int j=0; j< m; j++)
        {
            delete [] M3D[i][j];
        }
        delete[] M3D[i];
    }
    delete M3D;

    return;
}


///------------------------------------------------------------------------------
///---------------------- 2D Matrices  ------------------------------------------
///------------------------------------------------------------------------------


void ToPrintMatrix2D(float** M2D, int n, int m)
{
    for(int i=0; i<n; i++)
    {
        for(int j=0; j<m; j++)
        {
            cout<< M2D[i][j]<<" ";
        }
        cout<<endl;
    }
}


void ToInitMatrix2D(float** M2D, int n, int m)
{
    for(int i=0; i<n; i++)
    {
        for(int j=0; j<m; j++)
        {
            M2D[i][j]=0.0f;
        }
    }
}

void ToInitWithLabelMatrix2D(float** M2D, int n, int m, float label)
{
    for(int i=0; i<n; i++)
    {
        for(int j=0; j<m; j++)
        {
            M2D[i][j]=label;
        }
    }
}

float** ToCreateMatrix2D(int n, int m)
{
    float ** X = new float*[n];
    for(int i=0; i<n; i++)
    {
        X[i] = new float[m];
    }
    return X;
}

void ToEliminateMatrix2D(float** M2D, int n, int m)
{
    for(int i=0; i<n; i++)
    {
        delete[] M2D[i];
    }
    delete M2D;

    return;
}


///------------------------------------------------------------------------------
///---------------------- 1D Matrices = vectors ---------------------------------
///------------------------------------------------------------------------------


void ToPrintMatrix1D(float* M1D, int n)
{
    for(int i=0; i<n; i++)
    {
        cout<< M1D[i]<< " ";
    }
    cout<< endl;
}

void ToInitMatrix1D(float* M1D, int n)
{
    for(int i=0; i<n; i++)
    {
        M1D[i]=0;
    }
}

float* ToCreateMatrix1D(int n)
{
    float* X = new float[n];
    return X;
}

void ToEliminateMatrix1D(float* M1D, int n)
{
    //for(int i=0; i<n; i++)
    //{
    //delete[] M1D[i];
    //}
    delete[] M1D;
    return;
}







///------------------------------------------------------------------------------
///---------------------- 2D Mat Matrices  ---------------------------------------
///------------------------------------------------------------------------------




Mat** ToCreateMatrixMat2D(int n, int m)
{
    Mat ** X = new Mat*[n];
    for(int i=0; i<n; i++)
    {
        X[i] = new Mat[m];
    }
    return X;
}

void ToEliminateMatrixMat2D(Mat** M2D, int n, int m)
{
    for(int i=0; i<n; i++)
    {
        delete[] M2D[i];
    }
    delete M2D;

    return;
}
