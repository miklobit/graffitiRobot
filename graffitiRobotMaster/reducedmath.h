#ifndef MATH_H
#define MATH_H
///////////////////////////// Round Functions //////////////////////////////

float CEIL_FLOOR(float x, unsigned int8 n)
{
   float y, res;
   unsigned int16 l;
   int1 s;

   s = 0;
   y = x;

   if (x < 0)
   {
      s = 1;
      y = -y;
   }

   if (y <= 32768.0)
  res = (float)(unsigned int16)y;

 else if (y < 10000000.0)
   {
  l = (unsigned int16)(y/32768.0);
      y = 32768.0*(y/32768.0 - (float)l);
  res = 32768.0*(float)l;
  res += (float)(unsigned int16)y;
 }

 else
  res = y;

 y = y - (float)(unsigned int16)y;

 if (s)
  res = -res;

 if (y != 0)
 {
  if (s == 1 && n == 0)
   res -= 1.0;

  if (s == 0 && n == 1)
   res += 1.0;
 }
 if (x == 0)
    res = 0;

 return (res);
}

////////////////////////////////////////////////////////////////////////////
//   float round(float x)
////////////////////////////////////////////////////////////////////////////
// Description : rounds the number x.
// Date : N/A
//
float round(float x)
{
   float res = x - CEIL_FLOOR(x, 0);
   if (res > 0.5)
   {
     return CEIL_FLOOR(x, 1);
   }
   else
   {
    return CEIL_FLOOR(x, 0);
   }
}


////////////////////////////////////////////////////////////////////////////
//   float sqrt(float x)
////////////////////////////////////////////////////////////////////////////
// Description : returns the square root of x
// Date : N/A
//
float sqrt(float x)
{
   float y, res;
   #if defined(__PCD__)
   unsigned int16 data1,data2;
   #endif
   BYTE *p;

   #ifdef _ERRNO
   if(x < 0)
   {
      errno=EDOM;
   }
   #endif

   if( x<=0.0)
      return(0.0);

   y=x;
   
   #if !defined(__PCD__)
    p=&y;
   (*p)=(BYTE)((((unsigned int16)(*p)) + 127) >> 1);
   #endif
   
   #if defined(__PCD__)
    p = (((unsigned int8 *)(&y))+3);
    data1 = *(((unsigned int8 *)(&y))+3);
    data2 = *(((unsigned int8 *)(&y))+2);
    rotate_left(&data1,1);    
    if(bit_test(data2,7))    
    bit_set(data1,0);    
    data1 = ((data1+127) >>1);
    bit_clear(data2,7);
    if(bit_test(data1,0))
    bit_set(data2,7);
    data1 = data1 >>1;
    *(((unsigned int8 *)(&y))+3) = data1;
    *(((unsigned int8 *)(&y))+2) = data2;
    
  #endif

   do {
      res=y;
      y+=(x/y);

      #if !defined(__PCD__)
     (*p)--;
   #endif
   
   #if defined(__PCD__)
    data1 = *(((unsigned int8 *)(&y))+3);
    data2 = *(((unsigned int8 *)(&y))+2);
    rotate_left(&data1,1);
    if(bit_test(data2,7))
    bit_set(data1,0);    
    data1--;
    bit_clear(data2,7);
    if(bit_test(data1,0))
    bit_set(data2,7);
    data1 = data1 >>1;
    *(((unsigned int8 *)(&y))+3) = data1;
    *(((unsigned int8 *)(&y))+2) = data2;
    
  #endif
   } while(res != y);

   return(res);
}



