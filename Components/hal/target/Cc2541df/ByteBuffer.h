
#ifndef BYTEBUFFER_H
#define BYTEBUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */

//#include "comdef.h"
//#include "df_util.h"
//#include "hal_types.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * API FUNCTIONS
 */

// This method initializes the datastore of the buffer to a certain sizem the buffer should NOT be used before this call is made
extern void buffer_init(unsigned int buf_size);

// This method resets the buffer into an original state (with no data)	
extern void buffer_clear();

// This method fill the buffer with 0x41 data	
extern void buffer_fill();

// This releases resources for this buffer, after this has been called the buffer should NOT be used
extern void buffer_deAllocate();

// Returns how much space is left in the buffer for more data
extern int buffer_getSize();
	
// Returns the maximum capacity of the buffer
extern int buffer_getCapacity();

// This method returns the byte that is located at index in the buffer but doesn't modify the buffer like the get methods (doesn't remove the retured byte from the buffer)
extern char buffer_peek(unsigned int index);

//
// Put methods, either a regular put in back or put in front
// 
extern int buffer_putInFront(char in);
extern int buffer_put(char in);

extern int buffer_putIntInFront(int in);
extern int buffer_putInt(int in);

extern int buffer_putLongInFront(long in);
extern int buffer_putLong(long in);

extern int buffer_putFloatInFront(float in);
extern int buffer_putFloat(float in);

//
// Get methods, either a regular get from front or from back
// 
extern char buffer_get();
extern char buffer_getFromBack();

extern int buffer_getInt();
extern int buffer_getIntFromBack();

extern long buffer_getLong();	
extern long buffer_getLongFromBack();	

extern float buffer_getFloat();	
extern float buffer_getFloatFromBack();	


#ifdef __cplusplus
};
#endif

#endif

/**************************************************************************************************
*/

