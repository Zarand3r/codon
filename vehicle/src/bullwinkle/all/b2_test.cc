// TDD (P0): B2 / B2c byte-span contract.
#include "src/bullwinkle/all/B2.h"
#include "src/bullwinkle/all/B2c.h"

#include <cassert>
#include <cstdio>
#include <cstring>

using Drone::B2;
using Drone::B2c;

int main()
{
    // Default spans are empty.
    assert(B2c().buf() == nullptr && B2c().len() == 0);
    assert(B2().buf() == nullptr && B2().len() == 0);

    unsigned char buf[8] = {0, 1, 2, 3, 4, 5, 6, 7};

    // B2c borrows a const view.
    B2c cv(buf, sizeof(buf));
    assert(cv.buf() == buf && cv.len() == 8);
    assert(static_cast<const unsigned char *>(cv.buf())[3] == 3);

    // B2 is a mutable view; writes land in the backing buffer.
    B2 mv(buf, sizeof(buf));
    assert(mv.buf() == buf && mv.len() == 8);
    static_cast<unsigned char *>(mv.buf())[0] = 42;
    assert(buf[0] == 42);

    // B2 converts to B2c (mutable view → const view), preserving buf/len.
    B2c from_mut = mv;
    assert(from_mut.buf() == buf && from_mut.len() == 8);

    std::printf("b2_test: PASS\n");
    return 0;
}
