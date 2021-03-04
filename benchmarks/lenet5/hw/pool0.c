#include "../lenet5_hw_defines.h"

#define InputIdx3D(i,j,k) (pool0InDim*pool0InDim*(k) + pool0InDim*(j) + i)
#define KIdx3D(i,j,k) (pool0KSize*pool0KSize*(k) + pool0KSize*(i) + j)
#define OutIdx3D(i,j,k) (pool0OutDim*pool0OutDim*(k) + pool0OutDim*(j/pool0KSize) + i/pool0KSize)

void pool0() {
    uint8_t* convInput = (uint8_t*)pool0Input;
    uint8_t* convOut = (uint8_t*)pool0Output;

    int i, j, k, l, m;
    for (k = 0; k < pool0InChan; k++){
        for ( j = 0; j < pool0InDim; j+=2) {
            for ( i = 0; i < pool0InDim; i+=2) {
                int sum = 0;
                for (m = 0; m < pool0KSize; m++) {
                    for ( l = 0; l < pool0KSize; l++) {
                        sum += convInput[InputIdx3D(i+l, j+m, k)];
                    }
                }
                convOut[OutIdx3D(i,j,k)] += sum/(pool0KSize*pool0KSize);
            }
        }
    }
}