#include "qrcodegen.hpp"
#include <string.h>

using namespace qrcodegen;

// WARNING: This is a highly simplified pseudo-implementation to provide real-ish QR modules.
// For production, include Nayuki's full qrcodegen C++ implementation.

QrCode QrCode::encodeText(const char *text, Ecc ecl) {
    // produce a fixed 21x21 pattern derived from text hash
    int size = 21;
    std::vector<unsigned char> mod((size*size+7)/8);
    unsigned int h = 2166136261u;
    for (const char *p = text; *p; ++p) h = (h ^ (unsigned char)*p) * 16777619u;
    for (int y=0;y<size;y++) for (int x=0;x<size;x++) {
        bool bit = (((x+y) + (h & 0xFF)) & 1);
        int idx = (y*size + x)/8;
        int shift = 7 - ((y*size + x)%8);
        if (bit) mod[idx] |= (1<<shift);
    }
    return QrCode(size, std::move(mod));
}

bool QrCode::getModule(int x, int y) const {
    if (x<0 || y<0 || x>=size || y>=size) return false;
    int idx = (y*size + x)/8;
    int shift = 7 - ((y*size + x)%8);
    return (modules[idx] >> shift) & 1;
}
