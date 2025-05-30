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
MatrixXd X = MatrixXd::Random(5, 5);
MatrixXd A = X + X.transpose();
cout << "Here is a random symmetric matrix, A:" << endl << A << endl;
X = MatrixXd::Random(5, 5);
MatrixXd B = X * X.transpose();
cout << "and a random positive-definite matrix, B:" << endl << B << endl << endl;

GeneralizedSelfAdjointEigenSolver<MatrixXd> es(A, B);
cout << "The eigenvalues of the pencil (A,B) are:" << endl << es.eigenvalues() << endl;
cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;

double lambda = es.eigenvalues()[0];
cout << "Consider the first eigenvalue, lambda = " << lambda << endl;
VectorXd v = es.eigenvectors().col(0);
cout << "If v is the corresponding eigenvector, then A * v = " << endl << A * v << endl;
cout << "... and lambda * B * v = " << endl << lambda * B * v << endl << endl;

}
  return 0;
}
