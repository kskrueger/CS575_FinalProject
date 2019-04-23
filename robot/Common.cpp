//
// Created by Karter Krueger on 2019-04-12.
//

#include <chrono>
#include "Common.h"

//bool *RUNNING;

common::common(bool *running) {
    //RUNNING = running;
}

void common::wait(int milliseconds) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int elapsed = 0;

    while (elapsed < milliseconds/* && RUNNING*/) {
        std::chrono::steady_clock::time_point current = std::chrono::steady_clock::now();
        elapsed = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(current - begin).count());
    }
}
