// ============================================================================
// 文件名：test_utils.hpp
// 用途：CSV 数据驱动测试的公共工具（供 test_all_functions_csv.cpp 使用）。
// 结构：
//   - readCsv：读取 CSV → vector<map<列名,值>>；
//   - FuncTest：一个被测函数的描述（输入列/期望输出列/调用方式）；
//   - runCsvTests：读取输入 CSV、执行被测函数、对比期望、写出结果 CSV 并断言。
// 数据流：test_data/<name>.csv → invoke() → test_results_<name>.csv + gtest 断言。
// ============================================================================

#pragma once

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <cmath>
#include <stdexcept>
#include <gtest/gtest.h>

namespace test_utils
{

// 1) CSV を読み込んで、<列名→値> のマップのベクタを返す
// 读取 CSV：首行为列名，后续每行转成 <列名→double> 的 map

static std::vector<std::map<std::string, double>> readCsv(const std::string & path) {
  std::ifstream ifs(path);
  if (!ifs) throw std::runtime_error("Cannot open CSV: " + path);
  std::string line;
  std::vector<std::string> headers;
  // ヘッダ行
  if (std::getline(ifs, line)) {
    std::stringstream ss(line);
    std::string col;
    while (std::getline(ss, col, ',')) {
      headers.push_back(col);
    }
  }
  // データ行
  std::vector<std::map<std::string,double>> rows;
  while (std::getline(ifs, line)) {
    std::stringstream ss(line);
    std::string cell;
    std::map<std::string,double> row;
    for (size_t i = 0; i < headers.size() && std::getline(ss, cell, ','); ++i) {
      row[headers[i]] = std::stod(cell);
    }
    rows.push_back(row);
  }
  return rows;
}

// // 2) テスト対象関数を表す構造体
// 单个被测函数的测试描述：名称（对应 CSV 文件名）、输入列、期望输出列、
// 以及“CSV 一行 → 实际输出 vector”的调用函数

struct FuncTest {
  std::string name;
  std::vector<std::string> inCols;   // 入力カラム名
  std::vector<std::string> outCols;  // 期待値カラム名
  // CSV の１行 map<string,double> → 関数呼び出し → vector<double>（実際の戻り値）
  std::function<std::vector<double>(const std::map<std::string,double>&)> invoke;
};

// 3) テストランナー本体
// 运行器：对每个 FuncTest 读 CSV 逐行调用，比较期望与实际（容差 tol），
// 写出 test_results_<name>.csv 并给出 gtest 断言

inline void runCsvTests(
  const std::vector<FuncTest> &tests,
  double tol = 1e-3)
{
  for (auto &ft : tests) {
    // CSV 読み込み（カレントディレクトリ or WORKING_DIRECTORY に依存）
    auto rows = readCsv(ft.name + ".csv");

    // 出力ファイルを開く
    std::ofstream ofs("test_results_" + ft.name + ".csv");
    // ヘッダ行：期待値, 実際値, OK/NG
    for (size_t k = 0; k < ft.outCols.size(); ++k) {
      ofs << ft.outCols[k]         // 期待値カラム名
          << ",act_" << ft.outCols[k]; // 実際値カラム名
      if (k + 1 < ft.outCols.size()) ofs << ",";
    }
    ofs << ",OK\n";

    // 各行ループ
    for (size_t i = 0; i < rows.size(); ++i) {
      const auto &r = rows[i];
      auto result = ft.invoke(r);

      bool ok = true;
      for (size_t k = 0; k < ft.outCols.size(); ++k) {
        double expv = r.at(ft.outCols[k]);
        if (std::fabs(result[k] - expv) > tol) {
          ok = false;
        }
      }

      // CSV に書き出し
      for (size_t k = 0; k < ft.outCols.size(); ++k) {
        ofs << r.at(ft.outCols[k])     // 期待値
            << "," << result[k];       // 実測値
        if (k + 1 < ft.outCols.size()) ofs << ",";
      }
      ofs << "," << (ok ? "OK" : "NG") << "\n";

      // GTest のレポート
      EXPECT_TRUE(ok) << "[" << ft.name << "] row " << i;
    }

    ofs.close();
  }
}

} // namespace test_utils
