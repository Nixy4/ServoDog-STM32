#ifndef __CUTILS_H__
#define __CUTILS_H__

#include "stdint.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/**
 * @brief  将浮点数转换为整数(四舍五入)
 * @note   直接用进行类型转换,会直接截断小数部分
*/
int ftoi(float f);

/**
 * @brief  替换字符串中的指定字符
 * @param  src : 源字符串
 * @param  trg : 目标字符串
 * @param  rpl : 替换字符串
*/
void strrpl(char* src, const char* trg, const char* rpl);

/**
 * @brief  替换字符串中的指定字符
 * @param  src : 源字符串
 * @param  size: 缓冲区大小
 * @param  trg : 目标字符串
 * @param  rpl : 替换字符串
*/
void strnrpl(char* src, int size, const char* trg, const char* rpl);

extern 

#endif
