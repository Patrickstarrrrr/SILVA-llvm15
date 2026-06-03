#include <cstdio>

void foo(int* p) {
    *p = 1;
}

void bar(int* q) {
    *q = 2;
}

int main() {
    int x, y;
    foo(&x);
    bar(&y);
}
