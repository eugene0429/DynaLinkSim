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
MatrixXi mat(3, 3);
mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
cout << "Here is the matrix mat:\n" << mat << endl;

// The eval() solves the aliasing problem
mat.bottomRightCorner(2, 2) = mat.topLeftCorner(2, 2).eval();
cout << "After the assignment, mat = \n" << mat << endl;

}
  return 0;
}
