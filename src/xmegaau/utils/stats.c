/**
 * \file stats.c
 * \author tom.m
 *
 * \see stats.h.
 */

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

#include "project.h"
#include <math.h>
#include "stats.h"

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

#define SWAP(a, b) do { typeof(a) temp = a; a = b; b = temp; } while (0)

#ifndef PI
#define PI    3.14159265358979323846264338327950288
#endif

#define FRAME_SIZE 34


/************************************************************************/
/* Types.                                                               */
/************************************************************************/

/************************************************************************/
/* Private prototypes.                                                  */
/************************************************************************/

/*----------------------------------------------------------------------*/
void
WindowFunction
(
    void
);

/*----------------------------------------------------------------------*/
void
FastFourierTransform
(
    void
);

/*----------------------------------------------------------------------*/
void
LocateThatBitch
(
    float * arrray,
    int size,
    int left,
    int right,
    int index
);

/************************************************************************/
/* Constants.                                                           */
/************************************************************************/

/************************************************************************/
/* Private variables.                                                   */
/************************************************************************/

static float _data[FRAME_SIZE];
static STATS_Window_t _type;

/************************************************************************/
/* Macros.                                                              */
/************************************************************************/

/************************************************************************/
/* Implementations.                                                     */
/************************************************************************/

/*----------------------------------------------------------------------*/
void
STATS_Init
(
    void
)
{
    _type = 0;
    memset
    (
        _data,
        0.0,
        FRAME_SIZE*sizeof(float)
    );
}

/*----------------------------------------------------------------------*/
void
STATS_GetFastFourierTransform
(
    float * dst,
    float * src,
    int size
)
{
    /*
    Populate time-domain amplitude data locally.
    */
    memcpy
    (
        _data,
        src,
        size*sizeof(float)
    ); 
    /*
    Prepare signal for FFT.
    */
    WindowFunction();   
    /*
    Run FFT on the signal.
    */
    FastFourierTransform();
    /*
    Populate the frequency-domain spectrum data to-go.
    */
    memcpy
    (
        dst,
        _data,
        size*sizeof(float)
    ); 
}

/*----------------------------------------------------------------------*/
int
STATS_GetZeroCrossingRate
(
    float * array,
    int size
)
{
    int numZC = 0;
    for (int i=0; i<size-1; i++)
    {
        if
        (
            (
                (array[i] >= 0)
                &&
                (array[i+1] < 0)
            )
            ||
            (
                (array[i] < 0)
                &&
                (array[i+1] >= 0)
            )
        )
        {
            numZC++;
        }
    }
    return numZC;
}

/*----------------------------------------------------------------------*/
float
STATS_GetStandardDeviation
(
    float * array,
    int size
)
{
    float meanValue = 
        STATS_GetMean(array, size),
        diffSquare,
        stdDev;
    for (int i=0; i<size; i++)
    {
        diffSquare += pow(array[i]-meanValue,2);
    }
    if (size>0)
    {
        stdDev=sqrt(diffSquare/size);
    }
    return stdDev;
}

/*----------------------------------------------------------------------*/
float
STATS_GetCentroid
(
    float * array,
    int size
)
{
    float sumCentroid = 0, sumIntensities = 0, avgCentroid;
    for (int i=0; i<size; i++)
    {
        if (array[i] > 0)
        {
            sumCentroid+=i*array[i];
            sumIntensities+=array[i];
        }
    }
    avgCentroid=sumCentroid/sumIntensities;    
    return avgCentroid;
}

/*----------------------------------------------------------------------*/
float
STATS_GetMean
(
    float * array,
    int size
)
{
    return
    (
        STATS_GetSum(array, size) / size
    );
}

/*----------------------------------------------------------------------*/
float
STATS_GetSum
(
    float * array,
    int size
)
{
    float sum = 0.0;
    for (int i = 0 ; i < size; i++)
    {
        sum += array[i];
    }
    return sum;    
}

/*----------------------------------------------------------------------*/
int
STATS_GetMaxValueIndex
(
    float * array,
    int size
)
{
    int index = 0;
    float min = array[0];
    for (int i = 0; i < size; i++)
    {
        if (array[i] > min)
        {
            min = array[i];
            index = i;
        }
    }
    return index;
}

/*----------------------------------------------------------------------*/
int
STATS_GetMinValueIndex
(
    float * array,
    int size
)
{
    int index = 0;
    float min = array[0];
    for (int i = 0; i < size; i++) 
    {
        if (array[i] < min)
        {
            min = array[i];
            index = i;
        }
    }
    return index;    
}

/*----------------------------------------------------------------------*/
float
STATS_GetNthOrderedValue
(
    float * array,
    int size,
    int n,
    bool ascending
)
{
    int targetIndex;
    if (n > size) 
    {
        n = size;
    }
    targetIndex = (ascending)?n:size - n;
    /*
    This value is the value of the numKey-th element.
    */
    LocateThatBitch(array, size, 0, size - 1, targetIndex);  
    return array[targetIndex];
}

/*----------------------------------------------------------------------*/
void
STATS_GetNormalizedData
(
    float * dst,
    float * src,
    int size
)
{
    /*
    Normalization of data.
    Set max and min amplitudes.
    */
    float maxAmp = src[STATS_GetMaxValueIndex(src, size)];
    float minAmp = src[STATS_GetMinValueIndex(src, size)];
    /*
    Normalization.
    Avoiding divided by zero.
    */
    float minValidAmp = 0.00000000001F;
    float diff;
    if (minAmp == 0) 
    {
        minAmp = minValidAmp;
    }
    diff = log10f (maxAmp / minAmp); /* Perceptual difference. */
    for (int i = 0; i < size; i++) 
    {
        dst[i] = (src[i] < minValidAmp)? 0.0 : log10f(src[i]/minAmp) / diff;
    }    
}

/*----------------------------------------------------------------------*/
void
STATS_GetMovingAverageFilter
(
    float * dst,
    float * src,
    int size
)
{
    for (int i = 0; i < size; i++)
    {
        dst[i] = STATS_GetMean(src, i+1);
    }
}

/*----------------------------------------------------------------------*/
void
STATS_GetVectorMagnitude
(
    const STATS_VectorCoordinate_t *vector,
    float *result
)
{
    *result =
    sqrt
    (
        vector->x * vector->x
        +
        vector->y * vector->y
        +
        vector->z * vector->z
    );
}

/*----------------------------------------------------------------------*/
void
STATS_GetVectorDirection
(
    const STATS_VectorCoordinate_t *vector,
    STATS_VectorDirection_t *angle
)
{
    float magnitude = 0;
    STATS_GetVectorMagnitude(vector, &magnitude);
    angle->alpha = acos(vector->x / magnitude);
    angle->beta = acos(vector->y / magnitude);
    angle->theta = acos(vector->z / magnitude);
}

/*----------------------------------------------------------------------*/
void
STATS_GetRotationalVector
(
    const STATS_VectorCoordinate_t *vector,
    const STATS_VectorDirection_t *angle,
    STATS_VectorCoordinate_t *result
)
{
    float R[3][3] =
        {
            {
                cos(angle->alpha) * cos(angle->beta),
                cos(angle->alpha) * sin(angle->beta) * sin(angle->theta)
                -
                sin(angle->alpha) * cos(angle->theta),
                cos(angle->alpha) * sin(angle->beta) * cos(angle->theta)
                +
                sin(angle->alpha) * sin(angle->theta)
            },
            {
                sin(angle->alpha) * cos(angle->beta),
                sin(angle->alpha) * sin(angle->beta) * sin(angle->theta)
                +
                cos(angle->alpha) * cos(angle->theta),
                sin(angle->alpha) * sin(angle->beta) * cos(angle->theta)
                -
                cos(angle->alpha) * sin(angle->theta)
            },
            {
                -1 * sin(angle->beta),
                cos(angle->beta) * sin(angle->theta),
                cos(angle->beta) * cos(angle->theta)
            }
        };
    #if (0)
    float det =
        R[0][0]
        *
        (
            R[1][1]*R[2][2]-R[1][2]*R[2][1]
        )
        -
        R[0][1]
        *
        (
            R[1][0]*R[2][2]-R[1][2]*R[2][0]
        )
        +
        R[0][2]
        *
        (
            R[1][0]*R[2][1]-R[1][1]*R[2][0]
        );
    /* For check, should be +1 exactly. */
    mute((PSTR("#\tdet= %f\r\n"), det));
    #endif
    result->x = vector->x * R[0][0] + vector->y * R[0][1] + vector->z * R[0][2];
    result->y = vector->x * R[1][0] + vector->y * R[1][1] + vector->z * R[1][2];
    result->z = vector->x * R[2][0] + vector->y * R[2][1] + vector->z * R[2][2];  
}

/*----------------------------------------------------------------------*/
STATS_VectorCoordinate_t
STATS_GetVectorAddition
(
    STATS_VectorCoordinate_t vector1,
    STATS_VectorCoordinate_t vector2
)
{
  STATS_VectorCoordinate_t result =
    {
        vector1.x + vector2.x,
        vector1.y + vector2.y,
        vector1.z + vector2.z
    };    
    return result;
}

/*----------------------------------------------------------------------*/
void
STATS_GetVectorSubtraction
(
    const STATS_VectorCoordinate_t *vector1,
    const STATS_VectorCoordinate_t *vector2,
    STATS_VectorCoordinate_t *result
)
{
  
    result->x = vector1->x - vector2->x;
    result->y = vector1->y - vector2->y;
    result->z = vector1->z - vector2->z;
}

/*----------------------------------------------------------------------*/
void WindowFunction(void)
{
    /*
    Generate size window function values for index values 0 ... size - 1.
    */
    int m = FRAME_SIZE/2;
    float r;
    switch (_type) 
    {
        case STATS_Window_Bartlett:
            for (int n = 0; n < FRAME_SIZE; n++)
            {
                _data[n] *= (1.0f-abs(n-m)/m);
            }
            break;
        
        case STATS_Window_Hanning:
            r = PI/(m+1);
            for (int n = -m; n < m; n++)
            {
                _data[m+n] *= (0.5f+0.5f*cos(n*r));
            }
            break;
        
        case STATS_Window_Hamming:
            r = PI/m;
            for (int n = -m; n < m; n++)
            {
            _data[m+n] *= (0.54f+0.46f*cos(n*r));
            }
            break;
        
        case STATS_Window_Blackman:
            r = PI/m;
            for (int n = -m; n < m; n++)
            {
            _data[m+n] *= (0.42f+0.5f*cos(n*r)+0.08f*cos(2*n*r));
            }
            break;
        
        case STATS_Window_Rectangular:
            for (int n = 0; n < FRAME_SIZE; n++)
            {
                _data[n] *= 1.0f;
            }
            break;
            
        default:
            err
            ((
                PSTR
                (
                    "{"
                        "\"stats\":"
                        "{"
                            "\"window\":%u"
                        "}"
                    "}"
                ),
                _type
            ));
            break;
    }  
}

/*----------------------------------------------------------------------*/
void
FastFourierTransform()
{
    unsigned long
        n,
        mmax,
        m,
        j,
        istep,
        i;
    float
        wtemp,
        wr,
        wpr,
        wpi,
        wi,
        theta,
        tempr,
        tempi;
  
    /*
    The complex array is real+complex so the array 
    as a size n = 2* number of complex samples
    real part is the data[index] and 
    the complex part is the data[index+1].
    */
    float vector[2*FRAME_SIZE];

    /*
    Put the real array in a complex array
    the complex part is filled with 0's
    the remaining vector with no data is filled with 0's.
    */
    for(n = 0; n < FRAME_SIZE; n++)
    {
        vector[2*n] = (n<FRAME_SIZE)?_data[n]:0;
        vector[2*n+1] = 0;
    }

    /*
    Binary inversion (note that the indexes 
    start from 0 witch means that the
    real part of the complex is on the even-indexes 
    and the complex part is on the odd-indexes).
    */
    n = FRAME_SIZE<<1;
    j = 0;
    for (i=0;i<n/2;i+=2) 
    {
        if (j > i) 
        {
            /*
            Swap the real part.
            */
            SWAP(vector[j],vector[i]);
            /*
            Swap the complex part.
            */
            SWAP(vector[j+1],vector[i+1]);
            /*
            Checks if the changes occurs in the first half
            and use the mirrored effect on the second half. */
            if( (j/2) < (n/4) )
            {
                /*
                Swap the real part.
                */
                SWAP(vector[(n-(i+2))],vector[(n-(j+2))]);
                /*
                Swap the complex part.
                */
                SWAP(vector[(n-(i+2))+1],vector[(n-(j+2))+1]);
            }
        }
        m = n >> 1;
        while (m >= 2 && j >= m)
        {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    /*
    Danielson-Lanzcos routine.
    */
    mmax = 2;
    while (n > mmax) 
    {
        istep = mmax<<1;
        theta = -1*(2*PI/mmax);
        wtemp = sin(0.5*theta);
        wpr = -2.0*wtemp*wtemp;
        wpi = sin(theta);
        wr = 1.0;
        wi = 0.0;
        for (m = 1; m < mmax; m+=2)
        {
            for (i = m; i <= n; i+=istep)
            {
                j = i+mmax;
                tempr = wr*vector[j-1]-wi*vector[j];
                tempi = wr*vector[j]+wi*vector[j-1];
                vector[j-1] = vector[i-1]-tempr;
                vector[j] = vector[i]-tempi;
                vector[i-1] += tempr;
                vector[i] += tempi;
            }
            wr = (wtemp=wr)*wpr-wi*wpi+wr;
            wi = wi*wpr+wtemp*wpi+wi;
        }
        mmax = istep;
    }

    /*
    Populate the magnitude of the complex number.
    */
    for(i=0;i<n;i+=2)
    {
        _data[i] = sqrt(pow(vector[i],2)+pow(vector[i+1],2));
    }
}

/*----------------------------------------------------------------------*/
void
LocateThatBitch
(
    float * array,
    int size,
    int left,
    int right,
    int index
)
{
    int mid = (left + right) / 2;
    if (left < right) 
    {
        float s = array[mid];
        int i = left - 1;
        int j = right + 1;
        while (true) 
        {
            while (array[++i] < s);
            while (array[--j] > s);
            if (i >= j)
            {
                break;  
            }
            SWAP(array[i], array[j]);
        }
        LocateThatBitch
        (
            array,
            size,
            (i > index)?left:j + 1,
            (i > index)?i - 1:right,
            index
        );
    }
}
