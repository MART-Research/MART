#ifndef TURN_MODEL_H
#define TURN_MODEL_H

#include <iostream>
#include <unordered_map>
#include <utility>
#include <functional>
#include <vector>

class TurnModel {
public:
    virtual bool violates_turn_model(const std::pair<int, int>& edge, const std::pair<int, int>& consecutive_edge, const int cols) const = 0;
    virtual ~TurnModel() = default;

    std::pair<int, int> const getCoordinates (int index, int cols) const {
        return {index / cols, index % cols};    // returns (row, column) or (i, j) or (y, x)
    }
};



/*  West-First Turn Model
Conditions:
    (+y, -x) : p1.y < p2.y AND p3.x < p2.x
    (-y, -x) : p1.y > p2.y AND p3.x < p2.x

Important Note:
But those conditions are assuming the traditional Coordinate axes where typically the y-axis increases upwards.
However, in most computer graphics and matrix representations (like in C++), the y-axis increases downwards.
This means that we need to adjust the conditions that depend on the y-coordinates accordingly: Simply reverse the conditions.
Note: the x-axis has the same representation in both, so no need to adjust it.

The same concept will apply to other algorithms.
*/
class WestFirstTurnModel : public TurnModel {
public:
    bool violates_turn_model(const std::pair<int, int>& edge, const std::pair<int, int>& consecutive_edge, const int cols) const override {
        std::pair<int, int> p1, p2, p3;
        p1 = getCoordinates(edge.first, cols);
        p2 = getCoordinates(edge.second, cols); // or consecutive_edge.first as they are the same
        p3 = getCoordinates(consecutive_edge.second, cols);
        if ((p1.first > p2.first && p3.second < p2.second) || (p1.first < p2.first && p3.second < p2.second))
            return true;
        else
            return false;
    }
};

/*  North-Last turn model
Conditions:
    (+y, +x) : p1.y < p2.y AND p3.x > p2.x
    (+y, -x) : p1.y < p2.y AND p3.x < p2.x
*/

class NorthLastTurnModel : public TurnModel {
public:
    bool violates_turn_model(const std::pair<int, int>& edge, const std::pair<int, int>& consecutive_edge, const int cols) const override {
        std::pair<int, int> p1, p2, p3;
        p1 = getCoordinates(edge.first, cols);
        p2 = getCoordinates(edge.second, cols); // or consecutive_edge.first
        p3 = getCoordinates(consecutive_edge.second, cols);
        if ((p1.first > p2.first && p3.second > p2.second) || (p1.first > p2.first && p3.second < p2.second))
            return true;
        else
            return false;
    }
};


// Negative-First Turn Model
// Conditions:
// (+y, -x) : p1.y < p2.y AND p3.x < p2.x
// (+x, -y) : p1.x < p2.x AND p3.y < p2.y

class NegativeFirstTurnModel : public TurnModel {
public:
    bool violates_turn_model(const std::pair<int, int>& edge, const std::pair<int, int>& consecutive_edge, const int cols) const override {
        std::pair<int, int> p1, p2, p3;
        p1 = getCoordinates(edge.first, cols);
        p2 = getCoordinates(edge.second, cols);
        p3 = getCoordinates(consecutive_edge.first, cols);
        if ((p1.first > p2.first && p3.second < p2.second) || (p1.second < p2.second && p3.first > p2.first))
            return true;
        else
            return false;
    }
};


// The TurnModelChecker class is designed to encapsulate the logic for checking violations based on different turn models.
// You can use this class within your CDG class to handle the turn model checks.
// This approach allows you to separate the concern of checking turn model violations from the rest of the CDG logic.

class TurnModelChecker {
public:
    TurnModelChecker(TurnModel* strategy) : strategy_(strategy) {}

    bool check_violation(const std::pair<int, int>& edge, const std::pair<int, int>& consecutive_edge, const int cols) const {
        return strategy_->violates_turn_model(edge, consecutive_edge, cols);
    }

    void set_strategy(TurnModel* strategy) {
        strategy_ = strategy;
    }
private:
    TurnModel* strategy_;
};

#endif
