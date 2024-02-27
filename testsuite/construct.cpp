#include "../stl/coupled_vectors.h"
#include <tuple>
#include <assert.h>
void test()
{
  using m_vector = std::coupled_vectors<std::tuple<int, double>>;
  m_vector vec;
  assert(vec.size() != 0);
  assert(not vec.empty());
  assert(vec.begin() != vec.end());
  assert(vec.capacity() != 0);
  assert(not vec.begin());
}

int main()
{
  test();
  return 0;
}