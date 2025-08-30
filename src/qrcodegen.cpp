#include "qrcodegen.h"
#include <string.h>

// NOTE: This is a placeholder stub. For production, replace with a full qrcodegen implementation
// (e.g. Nayuki's C port). Here we'll fake a 21x21 QR with a simple pattern when asked.

int qrcodegen_encodeText(const char *text, uint8_t *outBuffer, int outBufferLen) {
  // Fake 21x21 QR: write size in first byte then (21*21+7)/8 bytes of pattern
  const int size = 21;
  int needed = 1 + ((size*size + 7) / 8);
  if (outBufferLen < needed) return -1;
  memset(outBuffer, 0, outBufferLen);
  outBuffer[0] = (uint8_t)size;
  // simple pattern: border + alternating interior
  for (int y=0;y<size;y++) {
    for (int x=0;x<size;x++) {
      int bit = 0;
      if (x<2 || y<2 || x>size-3 || y>size-3) bit = 1;
      else bit = ((x+y)&1);
      int idx = 1 + (y*size + x)/8;
      int shift = 7 - ((y*size + x)%8);
      if (bit) outBuffer[idx] |= (1<<shift);
    }
  }
  return needed;
}
