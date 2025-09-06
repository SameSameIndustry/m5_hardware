#include "m5.hpp"

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <sstream>
#include <iomanip>

namespace m5link {

// ---- ユーティリティ ----
static speed_t baud_from_int(int bps) {
  switch (bps) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return 0;
  }
}

M5SerialClient::M5SerialClient() {}
M5SerialClient::~M5SerialClient() { stop(); }

bool M5SerialClient::start(const std::string& port, int baudrate, size_t expected_joints) {
  if (running_) return true;
  dev_ = port;
  baud_ = baudrate;
  expected_joints_ = expected_joints;

  fd_ = openSerial(dev_, baud_);
  if (fd_ < 0) return false;

  stop_flag_ = false;
  running_ = true;
  rx_thread_ = std::thread(&M5SerialClient::rxLoop, this);
  tx_thread_ = std::thread(&M5SerialClient::txLoop, this);
  return true;
}

void M5SerialClient::stop() {
  if (!running_) return;
  stop_flag_ = true;

  // TX待ちを起こす
  { std::lock_guard<std::mutex> lk(tx_mtx_); }
  tx_cv_.notify_all();

  // ファイルディスクリプタを閉じるとrx select()が解除される
  closeSerial();

  if (rx_thread_.joinable()) rx_thread_.join();
  if (tx_thread_.joinable()) tx_thread_.join();

  running_ = false;
}

void M5SerialClient::sendSetPosition(const std::vector<double>& positions) {
  if (!running_) return;

  // CSV 生成（固定小数点、'.'使用）
  std::ostringstream oss;
  oss.imbue(std::locale::classic());               // 小数点を'.'に固定
  oss << "SET_POS," << positions.size();
  oss << std::fixed << std::setprecision(6);
  for (double v : positions) {
    oss << "," << v;
  }
  oss << "\n";
  const std::string msg = oss.str();

  // キュー投入（即return）
  {
    std::lock_guard<std::mutex> lk(tx_mtx_);
    tx_queue_.push_back(msg);
  }
  tx_cv_.notify_one();
}

void m5link::M5SerialClient::sendSetCommand(const std::vector<double>& positions,
                                            const std::vector<double>& efforts) {
  if (!running_) return;
  const size_t n = std::min(positions.size(), efforts.size());

  std::ostringstream oss;
  oss.imbue(std::locale::classic());
  oss << "SET_CMD," << n << std::fixed << std::setprecision(6);

  // まず position ベクトル N 個
  for (size_t i = 0; i < n; ++i) oss << "," << positions[i];

  // 続いて effort ベクトル N 個
  for (size_t i = 0; i < n; ++i) oss << "," << efforts[i];

  oss << "\n";
  const std::string msg = oss.str();

  {
    std::lock_guard<std::mutex> lk(tx_mtx_);
    tx_queue_.push_back(msg);
  }
  tx_cv_.notify_one();
}

bool M5SerialClient::tryGetLatestPositions(std::vector<double>& positions, std::chrono::steady_clock::time_point* stamp) {
  std::lock_guard<std::mutex> lk(state_mtx_);
  if (latest_pos_.empty()) return false;
  positions = latest_pos_;
  if (stamp) *stamp = latest_stamp_;
  return true;
}

// ---- 低レベルI/O ----
int M5SerialClient::openSerial(const std::string& dev, int baudrate) {
  int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    std::fprintf(stderr, "[M5] open('%s') failed: %s\n", dev.c_str(), std::strerror(errno));
    return -1;
  }

  termios tio{};
  if (tcgetattr(fd, &tio) < 0) {
    std::fprintf(stderr, "[M5] tcgetattr failed: %s\n", std::strerror(errno));
    ::close(fd);
    return -1;
  }

  cfmakeraw(&tio);

  speed_t spd = baud_from_int(baudrate);
  if (spd == 0) {
    std::fprintf(stderr, "[M5] unsupported baud: %d\n", baudrate);
    ::close(fd);
    return -1;
  }
  if (cfsetispeed(&tio, spd) < 0 || cfsetospeed(&tio, spd) < 0) {
    std::fprintf(stderr, "[M5] cfset[io]speed failed: %s\n", std::strerror(errno));
    ::close(fd);
    return -1;
  }

  // 8N1 / フロー制御OFF
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag &= ~CRTSCTS;                // HW flow off
  tio.c_iflag &= ~(IXON | IXOFF | IXANY); // SW flow off

  // 非ブロッキングreadはselectで見るのでVMIN/VTIMEは0
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tio) < 0) {
    std::fprintf(stderr, "[M5] tcsetattr failed: %s\n", std::strerror(errno));
    ::close(fd);
    return -1;
  }

  // オープン直後のゴミを捨てる
  tcflush(fd, TCIOFLUSH);
  return fd;
}

void M5SerialClient::closeSerial() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

// ---- スレッド ----
void M5SerialClient::rxLoop() {
  std::string line;
  line.reserve(256);
  char buf[256];

  while (!stop_flag_) {
    if (fd_ < 0) break;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200000; // 200ms刻みで停止フラグ確認

    int r = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (r < 0) {
      if (errno == EINTR) continue;
      std::fprintf(stderr, "[M5] select(rx) error: %s\n", std::strerror(errno));
      break;
    }
    if (r == 0) continue; // timeout: stopフラグ確認ループ

    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n < 0) {
      if (errno == EINTR) continue;
      std::fprintf(stderr, "[M5] read error: %s\n", std::strerror(errno));
      break;
    }
    if (n == 0) {
      // 相手切断の可能性
      continue;
    }

    for (ssize_t i = 0; i < n; ++i) {
      char c = buf[i];
      if (c == '\n') {
        // 完成行（末尾CR除去）
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) handleLine(line);
        line.clear();
      } else {
        if (line.size() < 1024) line.push_back(c);
        else line.clear(); // 異常な長さは破棄
      }
    }
  }
}

void M5SerialClient::txLoop() {
  while (!stop_flag_) {
    std::string msg;
    {
      std::unique_lock<std::mutex> lk(tx_mtx_);
      tx_cv_.wait(lk, [&]{ return stop_flag_ || !tx_queue_.empty(); });
      if (stop_flag_) break;
      msg.swap(tx_queue_.front());
      tx_queue_.pop_front();
    }
    if (fd_ < 0) continue;

    // write_all
    const char* p = msg.data();
    size_t left = msg.size();
    while (left > 0 && !stop_flag_) {
      ssize_t n = ::write(fd_, p, left);
      if (n < 0) {
        if (errno == EINTR) continue;
        std::fprintf(stderr, "[M5] write error: %s\n", std::strerror(errno));
        break;
      }
      left -= (size_t)n;
      p    += (size_t)n;
    }
    // 送信完了待ち（短い行なのでOK。長大行なら省略可）
    if (fd_ >= 0) ::tcdrain(fd_);
  }
}

// ---- パース ----
// 受信行の解釈（最小：STATE,N,rad...）
void M5SerialClient::handleLine(const std::string& line) {
  std::vector<std::string> tok;
  if (!splitCSV(line, tok) || tok.empty()) return;

  if (tok[0] == "STATE") {
    if (tok.size() < 2) return;
    unsigned N = 0;
    if (!parseUint(tok[1], N)) return;
    if (tok.size() != 2 + N) return;

    std::vector<double> pos;
    pos.reserve(N);
    for (unsigned i = 0; i < N; ++i) {
      double v;
      if (!parseDouble(tok[2 + i], v)) return;
      pos.push_back(v);
    }

    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      latest_pos_.swap(pos);
      latest_stamp_ = std::chrono::steady_clock::now();
    }
  }
  // 必要なら "VEL", "EFF" の行も追加実装可能
}

// カンマ区切り分割（単純版）
bool M5SerialClient::splitCSV(const std::string& s, std::vector<std::string>& out) {
  out.clear();
  std::string cur;
  for (char c : s) {
    if (c == ',') {
      out.emplace_back(std::move(cur));
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.emplace_back(std::move(cur));
  return true;
}

bool M5SerialClient::parseUint(const std::string& s, unsigned& v) {
  char* end = nullptr;
  errno = 0;
  unsigned long ul = std::strtoul(s.c_str(), &end, 10);
  if (errno || end == s.c_str() || *end != '\0' || ul > 0xFFFFFFFFul) return false;
  v = static_cast<unsigned>(ul);
  return true;
}

bool M5SerialClient::parseDouble(const std::string& s, double& v) {
  // ロケール非依存（classic）で '.' 小数点想定
  char* end = nullptr;
  errno = 0;
  v = std::strtod(s.c_str(), &end);
  if (errno || end == s.c_str() || *end != '\0') return false;
  return true;
}

} // namespace m5link
