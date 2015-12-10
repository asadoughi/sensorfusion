#include <iostream>

#include "vec.h"
#include "mat.h"

using namespace android;

int main(int argc, char* argv[])
{
  float aa[] = {1, 2, 3};
  float bb[] = {4, 5, 6};
  float cc[] = {7, 8, 9};
  vec3_t a(aa), b(bb), c(cc);

  mat33_t R;
  R << a << b << c;

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      std::cout << R[i][j] << std::endl;

  return 0;
}
