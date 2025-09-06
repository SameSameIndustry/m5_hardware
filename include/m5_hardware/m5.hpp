#pragma once
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <chrono>

namespace m5link {

// M5StackとのUARTリンク（テキストCSVプロトコル）
// 送信: "SET_POS,N,rad1,...,radN\n"
// 受信: "STATE,N,rad1,...,radN\n"
class M5SerialClient {
public:
  M5SerialClient();
  ~M5SerialClient();

  // UART開始（スレッド起動）。expected_jointsは受信側の検証に使用（0なら無検証）
  bool start(const std::string& port, int baudrate, size_t expected_joints = 0);
  void stop();

  bool isRunning() const noexcept { return running_; }

  // 非同期送信（即return）。角度[rad]をN個（N=joint数）送る
  void sendSetPosition(const std::vector<double>& positions);
  void sendSetCommand(const std::vector<double>& positions,
                      const std::vector<double>& efforts);

  // 最新の関節位置（受信"STATE"）を取得（即return）
  // 戻り値: 取得できたらtrue（positionsにコピー）、未更新ならfalse
  bool tryGetLatestPositions(std::vector<double>& positions, std::chrono::steady_clock::time_point* stamp = nullptr);

  //（必要なら）速度・努力を扱うAPIも後から足せます

private:
  // 低レベルI/O
  int  openSerial(const std::string& dev, int baudrate);
  void closeSerial();

  // スレッドループ
  void rxLoop();
  void txLoop();

  // パーサ
  void handleLine(const std::string& line);
  static bool splitCSV(const std::string& s, std::vector<std::string>& out);
  static bool parseUint(const std::string& s, unsigned& v);
  static bool parseDouble(const std::string& s, double& v);

private:
  // UART
  int fd_{-1};
  std::string dev_;
  int baud_{115200};

  // 状態キャッシュ
  mutable std::mutex state_mtx_;
  std::vector<double> latest_pos_;
  std::chrono::steady_clock::time_point latest_stamp_{};

  size_t expected_joints_{0};

  // TXキュー
  std::mutex tx_mtx_;
  std::condition_variable tx_cv_;
  std::deque<std::string> tx_queue_;

  // スレッド管理
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_flag_{false};
  std::thread rx_thread_;
  std::thread tx_thread_;
};

} // namespace m5link
