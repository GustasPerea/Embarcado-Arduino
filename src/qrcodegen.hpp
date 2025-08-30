/*
 * qrcodegen.hpp
 * Minimal inclusion of Nayuki's QrCode generator (C++) - trimmed to needed API.
 * License: MIT (from Nayuki). Include full upstream when using in production.
 */
#ifndef QRCODEGEN_HPP
#define QRCODEGEN_HPP

#include <vector>
#include <string>

namespace qrcodegen {

class QrCode {
public:
    enum class Ecc { QRECC_LOW, QRECC_MEDIUM, QRECC_QUARTILE, QRECC_HIGH };

    // Encodes text into a QR Code and returns an object.
    static QrCode encodeText(const char *text, Ecc ecl);

    int getSize() const { return size; }
    bool getModule(int x, int y) const;

private:
    int size;
    std::vector<unsigned char> modules; // row-major bits
    QrCode(int s, std::vector<unsigned char> &&m) : size(s), modules(m) {}
    friend class QrCodeEncoder;
};

}

#endif // QRCODEGEN_HPP
