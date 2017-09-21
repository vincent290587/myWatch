/******************************* SOURCE LICENSE *********************************
Copyright (c) 2015 MicroModeler.

A non-exclusive, nontransferable, perpetual, royalty-free license is granted to the Licensee to
use the following Information for academic, non-profit, or government-sponsored research purposes.
Use of the following Information under this License is restricted to NON-COMMERCIAL PURPOSES ONLY.
Commercial use of the following Information requires a separately executed written license agreement.

This Information is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

******************************* END OF LICENSE *********************************/

// A commercial license for MicroModeler DSP can be obtained at http://www.micromodeler.com/launch.jsp

#include "cmsis_filter.h"

#include <stdlib.h> // For malloc/free
#include <string.h> // For memset

float32_t filter1_coefficients[20] =
{
// Scaled for floating point

    0.5561084716141098, -1.1122169432282196, 0.5561084716141098, 1.7424183343315816, -0.7955152822547051,// b0, b1, b2, a1, a2
    0.5, -1, 0.5, 1.9120503151793817, -0.9422066680157611,// b0, b1, b2, a1, a2
    0.0625, 0.125, 0.0625, 1.4979686618898895, -0.6612014873672566,// b0, b1, b2, a1, a2
    0.0625, 0.125, 0.0625, 1.4942063767623008, -0.8229708544925012// b0, b1, b2, a1, a2

};


filter1Type *filter1_create( void )
{
	filter1Type *result = (filter1Type *)malloc( sizeof( filter1Type ) );	// Allocate memory for the object
	filter1_init( result );											// Initialize it
	return result;																// Return the result
}

void filter1_destroy( filter1Type *pObject )
{
	free( pObject );
}

 void filter1_init( filter1Type * pThis )
{
	arm_biquad_cascade_df1_init_f32(	&pThis->instance, filter1_numStages, filter1_coefficients, pThis->state );
	filter1_reset( pThis );

}

 void filter1_reset( filter1Type * pThis )
{
	memset( &pThis->state, 0, sizeof( pThis->state ) ); // Reset state to 0
	pThis->output = 0;									// Reset output

}

 int filter1_filterBlock( filter1Type * pThis, float * pInput, float * pOutput, unsigned int count )
{
	arm_biquad_cascade_df1_f32( &pThis->instance, pInput, pOutput, count );
	return count;

}


