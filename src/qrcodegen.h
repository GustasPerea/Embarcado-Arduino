/* Minimal qrcodegen wrapper (based on Project Nayuki public domain code) */
#ifndef QRCODEGEN_H
#define QRCODEGEN_H

#include <stdint.h>
#include <stddef.h>

// We'll expose a simple function that encodes text into a monochrome matrix.
// The implementation is a tiny subset adequate for our SVG/ASCII generation.

int qrcodegen_encodeText(const char *text, uint8_t *outBuffer, int outBufferLen);
// After encoding, caller can use returned size 'size' and read module (x,y) from outBuffer via bit test.
// For simplicity we'll pack bits row-wise per byte: outBuffer[(y*size + x)/8] & (1 << (7 - (x%8)))

#endif // QRCODEGEN_H
