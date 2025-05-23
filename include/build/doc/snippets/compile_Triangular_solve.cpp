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
Matrix3d m = Matrix3d::Zero();
m.triangularView<Eigen::Upper>().setOnes();
cout << "Here is the matrix m:\n" << m << endl;
Matrix3d n = Matrix3d::Ones();
n.triangularView<Eigen::Lower>() *= 2;
cout << "Here is the matrix n:\n" << n << endl;
cout << "And now here is m.inverse()*n, taking advantage of the fact that"
        " m is upper-triangular:\n"
     << m.triangularView<Eigen::Upper>().solve(n) << endl;
cout << "And this is n*m.inverse():\n" << m.triangularView<Eigen::Upper>().solve<Eigen::OnTheRight>(n);

}
  return 0;
}
