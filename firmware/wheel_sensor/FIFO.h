#ifndef FIFO_H_
#define FIFO_H_

#include <stdint.h>
#include "Mutex.h"

template <typename Type, int N, typename Size_t=uint8_t, typename Mutex_t=Mutex>
class FIFO
{
//variables
protected:
  Type              itsBuffer[N];    // allocated buffer
  volatile Size_t   itsBottom;    // position of oldest element
  volatile Size_t   itsTop;     // position of next free space
  volatile Size_t   itsCount;   // current count of elements in buffer
  Mutex_t           itsMux;

//functions
public:
  // Constructs FIFO
  FIFO()
  {
    itsMux.lock();
    itsBottom = 0;
    itsTop = 0;
    itsCount = 0;
    itsMux.unlock();
  }
  
  //adds one element
  //returns true if operation successful
  bool push(const Type& c)
  {
    bool res;
    itsMux.lock();
   
    Size_t count = itsCount;
    Size_t top = itsTop;
    
    if(count < N)
    {
      itsBuffer[top] = c;
      
      // Increment with wrap
      if((++top) >= N)
        top = 0;
        
      itsCount = (++count);
      itsTop = top;
      res = true;
    }
    else
    {
      // buffer is full, new elements are discarded
      res = false;
    }
    itsMux.unlock();
    
    return res;
  }
    
  //removes oldest item from buffer
  void pop()
  {
    itsMux.lock();
   
    Size_t count = itsCount;
    Size_t bottom = itsBottom;
    
    if(count > 0)//buffer not empty
    {
      // Increment bottom with wrap
      if((++bottom) >= N)
        bottom = 0;
        
      itsCount = (--count);
      itsBottom = bottom;
    }
    
    itsMux.unlock();
  }
    
  // Returns first inserted element and removes it from buffer.
  //WARNING- unsafe function, check if FIFO is not empty first.
  /*  usage:
    ...
    if(!buffer.isEmpty())
      c = buffer.get();
  */      
  Type get()
  {
    Type output;
    
    itsMux.lock();
    
    Size_t count = itsCount;
    Size_t bottom = itsBottom;
    
    // Read data
    output = itsBuffer[bottom];
    
    if(count > 0)//buffer not empty
    {
      // Increment bottom with wrap
      if((++bottom) >= N)
        bottom = 0;
      
      itsCount = (--count);
      itsBottom = bottom;
    }
    
    itsMux.unlock();
    
    return output;
  }
  
  // Returns last inserted element and removes it from buffer.
  //WARNING- unsafe function, check if FIFO is not empty first
  /*  usage:
    ...
    if(!buffer.isEmpty())
      c = buffer.getBack();
  */
  Type getBack()
  {
    Type output;
    
    itsMux.lock();
   
    Size_t count = itsCount;
    Size_t top = itsTop;
  
    if(count > 0)//buffer not empty
    {
      // Decrement top with wrap
      if(top == 0)
        top = N;
      
      itsCount = (--count);
      itsTop = (--top);
    }
    
    // Read data
    output = itsBuffer[top];
      
    itsMux.unlock();
    
    return output;
  }  
  
  /* Returns first inserted not taken element but not removes it.  
     WARNING- unsafe function, check if FIFO is not empty first
     usage:
    ...
    if(!buffer.isEmpty())
      c = buffer.first();
  */      
  inline Type first() const
  {
    Type output;
    
    itsMux.lock();
    output = itsBuffer[itsBottom];
    itsMux.unlock();
    
    return output;
  }
  
  /* Returns last inserted not taken element.  
     WARNING- unsafe function, check if FIFO is not empty first
     usage:
    ...
    if(!buffer.isEmpty())
      c = buffer.last();
  */      
  inline Type last() const
  {
    Type output;
    
    itsMux.lock();
    output = itsBuffer[itsTop];
    itsMux.unlock();
    
    return output;
  } 
  
  //copies data from FIFO to given array from oldest to newest
  //returns count of copied elements
  Size_t toArray(Type* array, Size_t capacity) const
  {
    Size_t count = Min(capacity, itsCount);
    
    itsMux.lock();
    
    const Type* src = itsBuffer + itsBottom; 
  
    for(Size_t i=count; i; --i)
    {
      *(array++) = *(src++);
      
      // Wrap around
      if(src >= itsBuffer + N)
        src = itsBuffer;
    }
    
    itsMux.unlock();
    
    return count;
  }
  
  //returns count of elements in buffer
  inline Size_t count() const { return itsCount; }
    
  //returns capacity of empty buffer
  inline Size_t capacity() const { return N; }
  
  //returns amount of free space in buffer
  inline Size_t freeSpace() const { return (N - itsCount); }
  
  //returns true, when the buffer is full
  inline bool isFull() const { return (itsCount == N); }
  
  //returns true, when buffer is empty
  inline bool isEmpty() const { return (itsCount == 0); }
  
  //clears all data in buffer
  void flush()
  {
    itsMux.lock();
    itsBottom = 0;
    itsTop = 0;
    itsCount = 0;
    itsMux.unlock();
  }
 
}; //FIFO_buffer

#endif
