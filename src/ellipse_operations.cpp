//
// Created by ian on 10/5/15.
//
#include "ellipse_operations.h"

#include "grasp_sampler.h"

namespace bachelors_final_project
{

/**
 * Parametric ecuation of an ellipse with center in (h,k)
 * x = h + a*cos(alpha)
 * y = k + b*sin(alpha)
 * tg(theta) = b/a*tg(alpha)
 * x = h + cos(theta)/sqrt(cos(theta)^2/a^2 + sin(theta)^2/b^2)
 * y = k + sin(theta)/sqrt(cos(theta)^2/a^2 + sin(theta)^2/b^2)
 **/
detection::EllipseOperations::EllipseOperations()
{
  number_of_points_ = 30;
  delta_theta = 0.001;
  number_integrals = round(/*2.0 * M_PI*/ M_PI / delta_theta);

  // Computation params
  nextPoint = 0;
  run = 0.0;
  theta = 0.0;
  computation_step = 0;
}

void detection::EllipseOperations::prepareComputation(double a, double b)
{
  a_ = a;
  b_ = b;
  circumference = computeCircumference(a, b, delta_theta, number_integrals);
  nextPoint = 0;
  run = 0.0;
  theta = 0.0;
  computation_step = 0;
}

bool detection::EllipseOperations::ellipsePointsLeft()
{
  return computation_step < number_integrals;
}

bool detection::EllipseOperations::getNewEllipsePoint(double height, PointT *point)
{
  bool found = false;
  // Calculate ellipse point
  while (computation_step < number_integrals && !found)
  {
    theta += delta_theta;
    double sub_integral = number_of_points_ * run / circumference;
    if ((int) sub_integral >= nextPoint)
    {
      point->x = (float) height;
      point->y = (float) (b_ * sin(theta));
      point->z = (float) (a_ * cos(theta));
      nextPoint++;
      found = true;
    }
    run += computeDPoint(a_, b_, theta);
    computation_step++;
  }
  return found;
}

double detection::EllipseOperations::computeCircumference(double mayor_axis, double minor_axis, double delta_theta,
                                                          double number_integrals)
{
  /* integrate over the elipse to get the circumference */
  double circumference = 0.0;
  double theta = 0.0;
  for (int i = 0; i < number_integrals; i++)
  {
    theta += i * delta_theta;
    circumference += computeDPoint(mayor_axis, minor_axis, theta);
  }
  return circumference;
}

double detection::EllipseOperations::computeDPoint(double a, double b, double theta)
{
  return sqrt(pow(a * sin(theta), 2.0) + pow(b * cos(theta), 2.0));
}


void detection::EllipseOperations::setNumberOfPoints(int number_of_points)
{
  number_of_points_ = number_of_points;
}
} // namespace bachelors_final_project
