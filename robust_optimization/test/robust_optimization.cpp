#include <random>
#include <robust_solving.hpp>
#include <ceres/ceres.h>
#include <internal/apply_trimmer.hpp>
#include "gtest/gtest.h"
#include <algorithm>
#include <map>
#include <iostream>
#include <iterator>

// Alias for residual map.
using IdResMap = std::map<int, double>;

namespace {

// Generates residuals using a normal distribution and clamps each value between min_val and max_val.
IdResMap makeResiduals(int start_id, int num, double mean, double dev, double min_val, double max_val) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> distribution(mean, dev);

    IdResMap out;
    for (int i = 0; i < num; ++i) {
        double val = distribution(gen);
#if __cplusplus >= 201703L
        val = std::clamp(val, min_val, max_val);
#else
        val = std::max(val, min_val);
        val = std::min(val, max_val);
#endif
        out[start_id + i] = val;
    }
    return out;
}

// Generates a dataset consisting of outliers and inliers.
IdResMap makeData() {
    double res_thres = 3.5;
    int num_outliers = 10;
    auto outliers = makeResiduals(0, num_outliers, 5.0, 1.0, res_thres + 0.1, 100.0);
    auto inliers = makeResiduals(num_outliers, 100, 0.0, 1.0, 0.0, res_thres - 0.1);
    IdResMap input = outliers;
    std::copy(inliers.cbegin(), inliers.cend(), std::inserter(input, input.end()));
    return input;
}

void printData(const IdResMap& data) {
    for (const auto& el : data) {
        std::cout << el.first << " " << el.second << "\n";
    }
    std::cout << std::endl;
}

void printData(const std::vector<int>& data) {
    for (const auto& el : data) {
        std::cout << el << " ";
    }
    std::cout << std::endl;
}

} // anonymous namespace

TEST(Trimmers, TrimmerFix) {
    auto input = makeData();
    printData(input);

    auto outliers = robust_optimization::getOutliers(input, robust_optimization::TrimmerType::Fix, 3.5);
    ASSERT_EQ(outliers.size(), 10);
    std::cout << "Trimmer Fix Outliers:" << std::endl;
    printData(outliers);
}

TEST(Trimmers, TrimmerQuantile) {
    auto input = makeData();
    auto outliers = robust_optimization::getOutliers(input, robust_optimization::TrimmerType::Quantile, 0.9);

    ASSERT_EQ(outliers.size(), 11);

    std::cout << "After Trimmer Quantile:" << std::endl;
    printData(outliers);
}

namespace {

struct TestFuncInlier {
    template <typename T>
    bool operator()(const T* const param, T* res) const {
        res[0] = T(3.0) * param[0];
        return true;
    }
    static ceres::CostFunction* Create() {
        return new ceres::AutoDiffCostFunction<TestFuncInlier, 1, 1>(new TestFuncInlier());
    }
};

struct TestFuncOutlier {
    template <typename T>
    bool operator()(const T* const /*param*/, T* res) const {
        res[0] = T(10.0);
        return true;
    }
    static ceres::CostFunction* Create() {
        return new ceres::AutoDiffCostFunction<TestFuncOutlier, 1, 1>(new TestFuncOutlier());
    }
};

} // anonymous namespace

TEST(Solve, trimmed) {
    ceres::Problem prob;
    int num_inliers = 90;
    int num_outliers = 10;
    double x = 2.0;
    robust_optimization::ResidualIds ids;
    for (int i = 0; i < num_inliers; ++i) {
        ceres::CostFunction* test_func = TestFuncInlier::Create();
        auto id = prob.AddResidualBlock(test_func, nullptr, &x);
        ids.push_back(std::make_pair(id, 1));
    }
    for (int i = 0; i < num_outliers; ++i) {
        ceres::CostFunction* test_func = TestFuncOutlier::Create();
        auto id = prob.AddResidualBlock(test_func, nullptr, &x);
        ids.push_back(std::make_pair(id, 1));
    }

    robust_optimization::TrimmerSpecification spec{robust_optimization::TrimmerType::Quantile, 0.9};
    robust_optimization::solveTrimmed(std::vector<int>{0, 2}, ids, spec, prob);

    ASSERT_NEAR(x, 0.0, 0.001);
}
