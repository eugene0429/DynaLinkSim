static bool eigen_did_assert = false;
#define eigen_assert(X)                                                                \
  if (!eigen_did_assert && !(X)) {                                                     \
    std::cout << "### Assertion raised in " << __FILE__ << ":" << __LINE__ << ":\n" #X \
              << "\n### The following would happen without assertions:\n";             \
    eigen_did_assert = true;                                                           \
  }

#include <iostream>
#include <cassert>
#include <Eigen/Eigen>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

using namespace Eigen;
using namespace std;

int main(int, char**) {
  cout.precision(3);
// intentionally remove indentation of snippet
{
Matrix3i m1;
m1 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
cout << m1 << endl << endl;
Matrix3i m2 = Matrix3i::Identity();
m2.block(0, 0, 2, 2) << 10, 11, 12, 13;
cout << m2 << endl << endl;
Vector2i v1;
v1 << 14, 15;
m2 << v1.transpose(), 16, v1, m1.block(1, 1, 2, 2);
cout << m2 << endl;

}
  return 0;
}
