/*
Arduino Buffered Serial
A library that helps establish buffered serial communication with a 
host application.
Copyright (C) 2010 Sigurður Örn Aðalgeirsson (siggi@media.mit.edu)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "OSAL.h"
#include "ByteBuffer.h"

#include "hal_led.h"


#ifdef __cplusplus
extern "C" {
#endif

char* buffer_data;

unsigned int capacity;
unsigned int position;
unsigned int length;

void buffer_init(unsigned int buf_length){
//	buffer_data = (char*)osal_mem_alloc(sizeof(char)*buf_length);
//	capacity = buf_length;
//	position = 0;
//	length = 0;
//	buffer_data = (char*)osal_mem_alloc(2048);
	buffer_data = (char*)osal_mem_alloc(sizeof(char)*buf_length);

        if (buffer_data == NULL)
        HalLedSet(HAL_LED_2, HAL_LED_MODE_ON );

	capacity = buf_length;
	position = 0;
	length = 0;

        //Full write for test
//        for (int i=0; i < 64; i++)
//        buffer_putInt(i);
        
}

void buffer_deAllocate(){
	osal_mem_free(buffer_data);
}

void buffer_clear(){
	position = 0;
	length = 0;
}

void buffer_fill(){
        for (unsigned int i=0; i < capacity; i++)
           buffer_data[i]=0x41;
}

int buffer_getSize(){
	return length;
}

int buffer_getCapacity(){
	return capacity;
}

char buffer_peek(unsigned int index){
	char b = buffer_data[(position+index)%capacity];
	return b;
}

int buffer_put(char in){
	if(length < capacity){
		// save buffer_data byte at end of buffer
		buffer_data[(position+length) % capacity] = in;
		// increment the length
		length++;
		return 1;
	}
	// return failure
	return 0;
}

int buffer_putInFront(char in){
	if(length < capacity){
		// save buffer_data byte at end of buffer
		if( position == 0 )
			position = capacity-1;
		else
			position = (position-1)%capacity;
		buffer_data[position] = in;
		// increment the length
		length++;
		return 1;
	}
	// return failure
	return 0;
}

char buffer_get(){
	char b = 0;


	if(length > 0){
		b = buffer_data[position];
		// move index down and decrement length
		position = (position+1)%capacity;
		length--;
	}

	return b;
}

char buffer_getFromBack(){
	char b = 0;
	if(length > 0){
		b = buffer_data[(position+length-1)%capacity];
		length--;
	}

	return b;
}

//
// Ints
//

int buffer_putIntInFront(int in){
    char *pointer = (char *)&in;
	buffer_putInFront(pointer[0]);	
	buffer_putInFront(pointer[1]);	
        return TRUE;
}

//int buffer_putInt(int in){
//    char *pointer = (char *)&in;
//	buffer_put(pointer[1]);	
//	buffer_put(pointer[0]);	
//        return TRUE;
//}
int buffer_putInt(int in){
    char *pointer = (char *)&in;
	buffer_put(pointer[0]);	
	buffer_put(pointer[1]);	
        return TRUE;
}


int buffer_getInt(){
	int ret;
    char *pointer = (char *)&ret;
	pointer[1] = buffer_get();
	pointer[0] = buffer_get();
	return ret;
}

int buffer_getIntFromBack(){
	int ret;
    char *pointer = (char *)&ret;
	pointer[0] = buffer_getFromBack();
	pointer[1] = buffer_getFromBack();
	return ret;
}

//
// Longs
//

int buffer_putLongInFront(long in){
    char *pointer = (char *)&in;
	buffer_putInFront(pointer[0]);	
	buffer_putInFront(pointer[1]);	
	buffer_putInFront(pointer[2]);	
	buffer_putInFront(pointer[3]);	
        return TRUE;
}

//int buffer_putLong(long in){
//    char *pointer = (char *)&in;
//	buffer_put(pointer[3]);	
//	buffer_put(pointer[2]);	
//	buffer_put(pointer[1]);	
//	buffer_put(pointer[0]);	
//        return TRUE;
//}
int buffer_putLong(long in){
    char *pointer = (char *)&in;
	buffer_put(pointer[0]);	
	buffer_put(pointer[1]);	
	buffer_put(pointer[2]);	
	buffer_put(pointer[3]);	
        return TRUE;
}


long buffer_getLong(){
	long ret;
    char *pointer = (char *)&ret;
	pointer[3] = buffer_get();
	pointer[2] = buffer_get();
	pointer[1] = buffer_get();
	pointer[0] = buffer_get();
	return ret;
}

long buffer_getLongFromBack(){
	long ret;
    char *pointer = (char *)&ret;
	pointer[0] = buffer_getFromBack();
	pointer[1] = buffer_getFromBack();
	pointer[2] = buffer_getFromBack();
	pointer[3] = buffer_getFromBack();
	return ret;
}


//
// Floats
//

int buffer_putFloatInFront(float in){
    char *pointer = (char *)&in;
	buffer_putInFront(pointer[0]);	
	buffer_putInFront(pointer[1]);	
	buffer_putInFront(pointer[2]);	
	buffer_putInFront(pointer[3]);	
        return TRUE;
}

int buffer_putFloat(float in){
    char *pointer = (char *)&in;
	buffer_put(pointer[3]);	
	buffer_put(pointer[2]);	
	buffer_put(pointer[1]);	
	buffer_put(pointer[0]);	
        return TRUE;
}

float buffer_getFloat(){
	float ret;
    char *pointer = (char *)&ret;
	pointer[3] = buffer_get();
	pointer[2] = buffer_get();
	pointer[1] = buffer_get();
	pointer[0] = buffer_get();
	return ret;
}

float buffer_getFloatFromBack(){
	float ret;
    char *pointer = (char *)&ret;
	pointer[0] = buffer_getFromBack();
	pointer[1] = buffer_getFromBack();
	pointer[2] = buffer_getFromBack();
	pointer[3] = buffer_getFromBack();
	return ret;
}

#ifdef __cplusplus
};
#endif

