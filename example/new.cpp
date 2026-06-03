#include <cstdio>

void foo(int* p, int* q) {
    *p = 1;
}

void bar(int* q, int* z) {
    *q = 2;
}

int main() {
    int z;
    int x, y;
    foo(&x, &z);
    bar(&y, &z);
}
