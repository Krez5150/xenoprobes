//
// Created by preed on 1/6/16.
//

#ifndef XENOPROBES_SOLUTION_H
#define XENOPROBES_SOLUTION_H

#include "probe_arrangement.h"
#include <vector> // Required for std::vector

class Solution {
public:
  Solution();

  // Objectives for Pareto optimization
  const std::vector<double>& getObjectiveValues() const { return objective_values_; }
  void setObjectiveValues(const std::vector<double>& objectives) { objective_values_ = objectives; }
  // Example: Add a single objective value
  void addObjectiveValue(double value) { objective_values_.push_back(value); }
  void clearObjectiveValues() { objective_values_.clear(); }


  int getAge() const;
  void setAge(int);

  double getScore() const; // This might represent a primary objective or an aggregate

  void randomize();
  void evaluate();
  void mutate(double rate);

  void printTotals() const;
  void printSetup() const;
  const ProbeArrangement &getSetup() const { return setup_; }
  void setSetup(const ProbeArrangement &setup) { setup_ = setup; }

  bool hasSameArrangement(const Solution &arr2) const;

  Solution findBestChild(size_t numOffsprings, double mutationRate) const;

  bool operator==(const Solution &b) const;
  bool operator<(const Solution &b) const;
  bool operator>(const Solution &b) const;

private:
  double score_ = 0; // Existing single score
  int age_ = 0;
  ProbeArrangement setup_;
  std::vector<double> objective_values_; // For multi-objective optimization
};

#endif // XENOPROBES_SOLUTION_H
