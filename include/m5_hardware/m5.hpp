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

// 受信スナップショット（必要な配列だけ埋まる）
struct JointStateSnapshot {
  std::vector<double> position;  // 受信できた場合のみサイズ>0
  std::vector<double> velocity;  // "
  std::vector<double> effort;    // "
  std::chrono::steady_clock::time_point stamp;
};

// M5StackとのUARTリンク（テキストCSVプロトコル）
// 送信: SET_POS,N,p0,...,pN-1\n  または  SET_CMD,N,p...,e...\n
// 受信: 以下のいずれか
//   STATE,N,p0,...,pN-1
//   STATE_VEL,N,v0,...,vN-1
//   STATE_EFF,N,e0,...,eN-1
//   STATE2,N,p0,...,pN-1,v0,...,vN-1  // STATE2 は position + velocity
//   STATE_FULL,N,p...,v...,e...
class M5SerialClient {
public:
  M5SerialClient();
  ~M5SerialClient();

  bool start(const std::string& port, int baudrate, size_t expected_joints = 0);
  void stop();
  bool isRunning() const noexcept { return running_; }

  // 送信：位置のみ（後方互換）
  void sendSetPosition(const std::vector<double>& positions);
  // 送信：位置＋努力（原子更新）
  void sendSetCommand(const std::vector<double>& positions,
                      const std::vector<double>& efforts);

  // 取得：最新の位置のみ（後方互換）
  bool tryGetLatestPositions(std::vector<double>& positions,
                             std::chrono::steady_clock::time_point* stamp = nullptr);

  // 取得：最新の状態（position/velocity/effortのうち受信できたもの）
  bool tryGetLatestState(JointStateSnapshot& out);

private:
  // 低レベルI/O
  int  openSerial(const std::string& dev, int baudrate);
  void closeSerial();

  // スレッド
  void rxLoop();
  void txLoop();

  // 受信行処理
  void handleLine(const std::string& line);

  // パース補助
  static bool splitCSV(const std::string& s, std::vector<std::string>& out);
  static bool parseUint(const std::string& s, unsigned& v);
  static bool parseDouble(const std::string& s, double& v);

private:
  // UART
  int fd_{-1};
  std::string dev_;
  int baud_{115200};

  // 状態キャッシュ（ロック保護）
  mutable std::mutex state_mtx_;
  std::vector<double> latest_pos_;
  std::vector<double> latest_vel_;
  std::vector<double> latest_eff_;
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
