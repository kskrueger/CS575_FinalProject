//
// Created by Karter Krueger on 2019-04-02.
//

#include <cstdio>
#include <cstdlib>

extern "C" {
#include "BNO055/pi-bno055/include/getbno055.h"
}
char senaddr[256] = "0x28";


int main() {
    get_i2cbus(senaddr);
    set_mode(ndof);
    struct bnoeul bnod;

    while (1) {
        get_eul(&bnod);
        printf("EUL %3.2f %3.2f %3.2f\n", bnod.eul_head, bnod.eul_roll, bnod.eul_pitc);
    }
}