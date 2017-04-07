/**
 * \file stats.h
 * \author tom.m
 *
 * Provides functionality for signal spectral analysis 
 */

#ifndef _STATS
#define _STATS

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

/************************************************************************/
/* Types.                                                               */
/************************************************************************/

typedef struct
{
    float alpha;
    float beta;
    float theta;
} STATS_VectorDirection_t;

typedef struct
{
    float x;
    float y;
    float z;
    uint32_t count;
} STATS_VectorCoordinate_t;

typedef enum
{
    STATS_Window_Bartlett,
    STATS_Window_Hanning,
    STATS_Window_Hamming,
    STATS_Window_Blackman,
    STATS_Window_Rectangular
} STATS_Window_t;

/************************************************************************/
/* Public prototypes.                                                   */
/************************************************************************/

/**
 * Initialized the discrete fourier transform interface.
 */
/*----------------------------------------------------------------------*/
void
STATS_Init
(
    void
);

/*----------------------------------------------------------------------*/
void
STATS_GetFastFourierTransform
(
    float * dst, 
    float * src,
    int size
);

/*----------------------------------------------------------------------*/
int
STATS_GetZeroCrossingRate
(
    float * array,
    int size
);

/*----------------------------------------------------------------------*/
float
STATS_GetStandardDeviation
(
    float * array,
    int size
);

/*----------------------------------------------------------------------*/
float
STATS_GetCentroid
(
    float * array,
    int size
);

/*----------------------------------------------------------------------*/
float
STATS_GetMean
(
    float * array,
    int size
);

/*----------------------------------------------------------------------*/
float
STATS_GetSum
(
    float * array,
    int size
);

/*----------------------------------------------------------------------*/
int
STATS_GetMaxValueIndex
(
    float * array,
    int size
);

/*----------------------------------------------------------------------*/
int
STATS_GetMinValueIndex
(
    float * array,
    int size
);

/*----------------------------------------------------------------------*/
float
STATS_GetNthOrderedValue
(
    float * array,
    int size,
    int n,
    bool ascending
);

/*----------------------------------------------------------------------*/
void
STATS_GetNormalizedData
(
    float * dst, 
    float * src, 
    int size
);

/*----------------------------------------------------------------------*/
void
STATS_GetMovingAverageFilter
(
    float * dst, 
    float * src, 
    int size
);

/*----------------------------------------------------------------------*/
void
STATS_GetVectorMagnitude
(
    const STATS_VectorCoordinate_t *vector,
    float *result
);

/*----------------------------------------------------------------------*/
void
STATS_GetVectorDirection
(
    const STATS_VectorCoordinate_t *vector,
    STATS_VectorDirection_t *angle
);

/*----------------------------------------------------------------------*/
void
STATS_GetRotationalVector
(
    const STATS_VectorCoordinate_t *vector,
    const STATS_VectorDirection_t *angle,
    STATS_VectorCoordinate_t *result
);

/*----------------------------------------------------------------------*/
STATS_VectorCoordinate_t
STATS_GetVectorAddition
(
    STATS_VectorCoordinate_t vector1,
    STATS_VectorCoordinate_t vector2
);

/*----------------------------------------------------------------------*/
void
STATS_GetVectorSubtraction
(
    const STATS_VectorCoordinate_t *vector1,
    const STATS_VectorCoordinate_t *vector2,
    STATS_VectorCoordinate_t *result
);

/************************************************************************/
/* Public variables.                                                    */
/************************************************************************/

/************************************************************************/
/* Public Macros.                                                       */
/************************************************************************/

#endif  /* _STATS */