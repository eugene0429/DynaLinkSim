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
int rows = 5, cols = 5;
MatrixXf m(rows, cols);
m << (Matrix3f() << 1, 2, 3, 4, 5, 6, 7, 8, 9).finished(),
    MatrixXf::Zero(3, cols - 3), MatrixXf::Zero(rows - 3, 3), MatrixXf::Identity(rows - 3, cols - 3);
cout << m;

}
  return 0;
}
