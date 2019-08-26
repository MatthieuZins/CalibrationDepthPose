#ifndef MATCHINGMATRIX_H
#define MATCHINGMATRIX_H

#include <Eigen/Dense>
#include <iostream>

namespace CalibrationDepthPose
{

/// This struct define a strategy to use for point clouds matching
struct PairsMatchingStrategy
{
  enum class Strategy
  {
    ALL,  // match each point cloud with all the other point cloud
    N_CONSECUTIVES_WITH_LOOP,   // match each point cloud with the N next point clouds (with loop)
    N_CONSECUTIVES_NO_LOOP,     // match each point cloud with the N next point clouds (no loop)
  } strategy = Strategy::N_CONSECUTIVES_WITH_LOOP;

  int nbNeighbours = 1;     // number of successive point clouds to match
};

/// print a matching strategy
std::ostream& operator <<(std::ostream& os, PairsMatchingStrategy const& strategy);


/// Dynamic matrix of bool
using BooleanMatrix = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;


/// This class represents how point clouds are matched
class MatchingMatrix
{
public:
  MatchingMatrix(int n);

  // Add a new match between point clouds
  void addMatch(int a, int b, bool reciprocal = false) {
    matrix(a, b) = true;
    matrix(b, a) |= reciprocal;
  }

  // Clear all matches
  void erase() {
    matrix = BooleanMatrix::Zero(matrix.rows(), matrix.cols());
  }

  // Set the matrix
  void setMatrix(BooleanMatrix const& mat) {
    if (mat.rows() == matrix.rows() && mat.cols() == matrix.cols()) {
      matrix = mat;
    } else {
      std::cerr << "setMatrix(mat): mat has not the correct dimension" << std::endl;
    }
  }

  // Accessor to individual matches
  bool operator()(int i, int j) const {
    return matrix(i, j);
  }

  // Get the size of the matrix (i.e the number of point clouds)
  unsigned int getSize() const {
    return matrix.cols();
  }

  // Fill the matrix with a predefined matching strategy
  void setFromPredefinedMatchingStrategy(PairsMatchingStrategy const& strategy);

  // Return all matches in the form of a list of index pairs
  std::vector<std::pair<size_t, size_t> > getPairs() const;


  friend std::ostream& operator <<(std::ostream& os, MatchingMatrix const& matrix);


private:
  BooleanMatrix matrix;
};

// Print the matrix
std::ostream& operator <<(std::ostream& os, MatchingMatrix const& matrix);

} // namespace CalibrationDepthPose

#endif // MATCHINGMATRIX_H
