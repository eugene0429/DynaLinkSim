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
Matrix4d A = Matrix4d::Random(4, 4);
cout << "Here is a random 4x4 matrix:" << endl << A << endl;
HessenbergDecomposition<Matrix4d> hessOfA(A);
Matrix4d pm = hessOfA.packedMatrix();
cout << "The packed matrix M is:" << endl << pm << endl;
cout << "The upper Hessenberg part corresponds to the matrix H, which is:" << endl << hessOfA.matrixH() << endl;
Vector3d hc = hessOfA.householderCoefficients();
cout << "The vector of Householder coefficients is:" << endl << hc << endl;

}
  return 0;
}
