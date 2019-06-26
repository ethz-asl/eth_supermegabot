#include <std_utils/std_utils.hpp>

#include <iostream>

#define TYPE_LIST \
double, \
int

#define _STATIC_CAST(e) \
std::cout << static_cast<e>(3.14) << std::endl;

#define STATIC_CAST(...) \
EVAL(MAP(_STATIC_CAST, __VA_ARGS__))

int main() {
  STATIC_CAST(TYPE_LIST);
  return 0;
}