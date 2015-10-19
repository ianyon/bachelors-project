//
// Created by ian on 10/5/15.
//

#ifndef BACHELORS_FINAL_PROJECT_ELLIPSEOPERATIONS_H
#define BACHELORS_FINAL_PROJECT_ELLIPSEOPERATIONS_H

#include "definitions.h"

namespace bachelors_final_project
{
namespace detection
{

class EllipseOperations
{
public:
  EllipseOperations();

  void prepareComputation(double a, double b);

  bool ellipsePointsLeft();

  bool getNewEllipsePoint(double, Point *);

  double computeCircumference(double mayor_axis, double minor_axis, double delta_theta,
                              double number_integrals);

  double computeDPoint(double a, double b, double theta);

  double delta_theta;
  int number_of_points_;
  double number_integrals;

  // Computation vars
  double a_;
  double b_;
  double circumference;
  int nextPoint;
  double run;
  double theta;
  int computation_step;

  void setNumberOfPoints(int number_of_points);


};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_ELLIPSEOPERATIONS_H
