#include "CUtils.h"

int ftoi(float f)
{
  return (int)(f + 0.5f);
}

void strrpl(char* src, const char* trg, const char* rpl)
{
  char buffer[BUFSIZ];
  char *insert_point = &buffer[0];
  const char *tmp = src;
  size_t trg_len = strlen(trg);
  size_t rpl_len = strlen(rpl);

  while (1) {
    const char *p = strstr(tmp, trg);

    // walked past last occurrence of trg; copy remaining part
    if (p == NULL) {
      strcpy(insert_point, tmp);
      break;
    }

    // copy part before trg
    memcpy(insert_point, tmp, p - tmp);
    insert_point += p - tmp;

    // copy rplacement string
    memcpy(insert_point, rpl, rpl_len);
    insert_point += rpl_len;

    // adjust pointers, move on
    tmp = p + trg_len;
  }

  // write altered string back to src
  strcpy(src, buffer);
}

void strnrpl(char* src, int size, const char* trg, const char* rpl)
{
  char* buffer = (char*)malloc(size);
  if (buffer == NULL) {
    return;
  }
  if(strlen(src)>size){
    return;
  }

  char *insert_point = buffer;
  const char *tmp = src;
  size_t trg_len = strlen(trg);
  size_t rpl_len = strlen(rpl);

  while (1) {
    const char *p = strstr(tmp, trg);
    if (p == NULL) {
      strcpy(insert_point, tmp);
      break;
    }
    memcpy(insert_point, tmp, p - tmp);
    insert_point += p - tmp;
    memcpy(insert_point, rpl, rpl_len);
    insert_point += rpl_len;
    tmp = p + trg_len;
  }
  strcpy(src, buffer);
}
