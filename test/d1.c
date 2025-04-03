#include "stdio.h"

//基本数据类型
char c = 'A';
short s = 1;
int i = 2;
long l = 3;
float f = 4.0;
double d = 5.0;

signed char sc = 6; // 8Bit -128~127 -2^7~2^7-1
signed short s = 1; // 16Bit -32768~32767 -2^15~2^15-1
signed int i = 2;   // 32Bit -2147483648~2147483647 -2^31~2^31-1
signed long l = 3;

unsigned char uc = 6;  // 8Bit 0~255 2^8-1
unsigned short us = 1; // 16Bit 0~65535 2^16-1
unsigned int ui = 2;   // 32Bit 0~4294967295 2^32-1
unsigned long ul = 3; 

//标准数据类型
#include "stdint.h"

uint8_t u8 = 1;
uint16_t u16 = 2;
uint32_t u32 = 3;

//构造数据类型

//数组
char string1[5];
char string2[] = {'H', 'e', 'l', 'l', 'o'}; // len = 5
char string3[] = "Hello";// { 'H', 'e', 'l', 'l', 'o', '\0' } len = 6
char string4[6] = "Hello";

#include "string.h"
uint16_t string3_len = strlen(string1); // 5
uint16_t string3_size = sizeof(string1); // 6

void fnc1()
{
  if( strcmp(string3,string4) == NULL )
  {

  }

  char cmd[11] = "LED RED ON";

  if( strstr(cmd, "LED") != NULL )
  {
    //控制LED灯
    if(strstr(cmd, "RED") != NULL)
    {
      //控制红灯
    }
  }

  char a = cmd[0];
}

//结构体
struct student
{
  char name[10];
  int age;
  float score;
};

typedef struct student stu_t;

typedef struct
{
  char name[10];
  int age;
  float score;
}teh_t;

void fnc2()
{
  stu_t gnq = {"gnq", 18, 100.0};
  gnq.age = 19;
}

stu_t* stu = NULL; // 32bit uint32_t 

void fnc3(stu_t* stu)
{
  stu->age = 20;
}

int main()
{
  return 0;
}