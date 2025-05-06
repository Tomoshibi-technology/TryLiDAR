#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>    // ファイル制御 (open)
#include <termios.h>  // POSIX ターミナル制御
#include <unistd.h>   // UNIX 標準関数 (read, write, close, usleep)
#include <errno.h>    // エラー番号
#include <signal.h>   // シグナルハンドリング (sigaction)
#include <stdlib.h>   // exit
#include <sys/time.h> // gettimeofday, timersub (タイムアウト処理用)

// --- 定数定義 ---
// RPLiDAR コマンド
const uint8_t RPLIDAR_CMD_STOP = 0x25;
const uint8_t RPLIDAR_CMD_SCAN = 0x20;
const uint8_t RPLIDAR_CMD_FORCE_SCAN = 0x21; // 通常のSCANを推奨
const uint8_t RPLIDAR_CMD_RESET = 0x40;
const uint8_t RPLIDAR_CMD_GET_INFO = 0x50;
const uint8_t RPLIDAR_CMD_GET_HEALTH = 0x52;
const uint8_t RPLIDAR_CMD_SYNC_BYTE = 0xA5;

// タイミング関連 (マイクロ秒)
const useconds_t WAIT_AFTER_CMD_US = 2000;      // コマンド送信後の待機
const useconds_t WAIT_AFTER_STOP_US = 10000;    // STOPコマンド後の待機
const useconds_t WAIT_MOTOR_STABLE_US = 200000; // モーター安定待ち (必要に応じて調整)
const useconds_t READ_TIMEOUT_US = 1000000;     // 応答デスクリプタ読み取りタイムアウト (1秒)
const useconds_t READ_LOOP_WAIT_US = 1000;      // データ読み取りループでの待機 (CPU負荷軽減)

// データサイズ
#define RPLIDAR_CMD_LEN 2
#define SCAN_RESPONSE_DESCRIPTOR_SIZE 7
#define SCAN_DATA_PACKET_SIZE 5

// SCAN応答デスクリプタの期待値
const uint8_t EXPECTED_SCAN_DESCRIPTOR[SCAN_RESPONSE_DESCRIPTOR_SIZE] = {
    RPLIDAR_CMD_SYNC_BYTE, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};

// --- グローバル変数 ---
volatile sig_atomic_t stop_flag = 0; // Ctrl+Cハンドラ用

// --- 関数プロトタイプ宣言 ---
void handle_sigint(int sig);
int setup_serial_port(const char *portname, int speed);
int set_interface_attribs(int fd, int speed);
int send_command(int fd, uint8_t cmd);
ssize_t read_with_timeout(int fd, uint8_t *buf, size_t n, useconds_t timeout_us);

// --- データパケット構造体 ---
typedef struct
{
  uint8_t sync_quality;      // Sync bit (LSB=1), Inverse Sync bit (LSB+1=0), Quality (6 bits MSB)
  uint8_t angle_q6_checkbit; // Angle (lower 7 bits = LSB), Check bit (MSB=1) <- Check bitは旧仕様？ 通常は無視
  uint8_t angle_q14;         // Angle (upper 8 bits = MSB)
  uint8_t distance_q2_lsb;   // Distance (lower 8 bits)
  uint8_t distance_q2_msb;   // Distance (upper 8 bits)
} __attribute__((packed)) rplidar_response_measurement_node_t;

int main()
{
  const char *portname = "/dev/ttyAMA1"; // 環境に合わせて変更
  int fd = -1;                           // ファイルディスクリプタを初期化

  // --- シリアルポート設定 ---
  fd = setup_serial_port(portname, B115200);
  if (fd < 0)
  {
    return -1;
  }

  // --- Ctrl+C ハンドラ設定 (sigactionを使用) ---
  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = handle_sigint;
  sigemptyset(&sa.sa_mask); // ハンドラ実行中にブロックするシグナルはない
  sa.sa_flags = 0;          // 必要に応じて SA_RESTARTなどを追加
  if (sigaction(SIGINT, &sa, NULL) == -1)
  {
    perror("Error setting SIGINT handler");
    close(fd);
    return -1;
  }
  printf("Ctrl+C handler set. Press Ctrl+C to stop.\n");

  // --- LiDAR制御 ---
  printf("Sending STOP command...\n");
  if (send_command(fd, RPLIDAR_CMD_STOP) < 0)
  {
    close(fd);
    return -1;
  }
  usleep(WAIT_AFTER_STOP_US);
  tcflush(fd, TCIOFLUSH); // STOP後の応答などをクリア (念のため)
  printf("STOP command sent.\n");

  // --- モーター安定待ち ---
  printf("Waiting for motor to stabilize...\n");
  usleep(WAIT_MOTOR_STABLE_US);

  // --- スキャン開始 ---
  printf("Sending SCAN command...\n");
  if (send_command(fd, RPLIDAR_CMD_SCAN) < 0)
  {
    close(fd);
    return -1;
  }

  // --- 応答デスクリプタ読み取り & 検証 ---
  uint8_t descriptor_buf[SCAN_RESPONSE_DESCRIPTOR_SIZE];
  printf("Reading SCAN response descriptor...\n");
  ssize_t desc_read = read_with_timeout(fd, descriptor_buf, SCAN_RESPONSE_DESCRIPTOR_SIZE, READ_TIMEOUT_US);

  if (desc_read == (ssize_t)SCAN_RESPONSE_DESCRIPTOR_SIZE)
  {
    if (memcmp(descriptor_buf, EXPECTED_SCAN_DESCRIPTOR, SCAN_RESPONSE_DESCRIPTOR_SIZE) == 0)
    {
      printf("Scan response descriptor OK.\n");
    }
    else
    {
      fprintf(stderr, "Error: Invalid scan response descriptor received: ");
      for (size_t i = 0; i < SCAN_RESPONSE_DESCRIPTOR_SIZE; ++i)
      {
        fprintf(stderr, "0x%02X ", descriptor_buf[i]);
      }
      fprintf(stderr, "\n");
      send_command(fd, RPLIDAR_CMD_STOP); // 念のためSTOP送信
      close(fd);
      return -1;
    }
  }
  else
  {
    fprintf(stderr, "Error reading scan response descriptor (read %zd bytes, expected %zu)\n",
            desc_read, SCAN_RESPONSE_DESCRIPTOR_SIZE);
    if (desc_read < 0 && errno != ETIME)
    { // ETIMEはread_with_timeout内でのタイムアウト
      perror(" Read error");
    }
    send_command(fd, RPLIDAR_CMD_STOP); // 念のためSTOP送信
    close(fd);
    return -1;
  }

  // --- スキャンデータ読み取りループ（パケット単位）---
  printf("Starting to read scan data (packet-based)...\n");
  rplidar_response_measurement_node_t node;
  uint8_t packet[SCAN_DATA_PACKET_SIZE];
  // …スキャンデータ読み取りループ（パケット単位）…
  while (!stop_flag)
  {
    ssize_t rd = read(fd, packet, SCAN_DATA_PACKET_SIZE);
    if (rd != (ssize_t)SCAN_DATA_PACKET_SIZE)
      continue;

    // 全パケットから角度・距離を計算
    memcpy(&node, packet, SCAN_DATA_PACKET_SIZE);
    uint8_t quality = node.sync_quality >> 2;
    uint8_t start_bit = node.sync_quality & 0x01; // 回転開始の目印として利用可
    uint16_t angle_raw = ((uint16_t)node.angle_q14 << 7) | (node.angle_q6_checkbit & 0x7F);
    float angle_deg = angle_raw / 64.0f;
    uint16_t dist_raw = ((uint16_t)node.distance_q2_msb << 8) | node.distance_q2_lsb;
    float distance_mm = dist_raw / 4.0f;

    // 距離が0mmでない場合のみ出力
    if (distance_mm != 0.0f)
    {
      printf("%s Theta: %07.2f Deg, Dist: %08.2f mm, Q: %d\n",
             start_bit ? "S" : " ",
             angle_deg, distance_mm, quality);
    }
  }

  // --- 終了処理 ---
  printf("\nStopping scan...\n");
  if (send_command(fd, RPLIDAR_CMD_STOP) < 0)
  {
    fprintf(stderr, "Warning: Failed to send STOP command during shutdown.\n");
  }
  usleep(WAIT_AFTER_STOP_US);

  if (fd >= 0)
  {
    close(fd);
    printf("Serial port closed.\n");
  }

  return 0;
}

// --- 関数定義 ---

// Ctrl+C ハンドラ
void handle_sigint(int sig)
{
  (void)sig;
  stop_flag = 1;
}

// シリアルポートのオープンと設定を行うヘルパー関数
int setup_serial_port(const char *portname, int speed)
{
  // O_RDWR: 読み書きモード
  // O_NOCTTY: このプロセスを制御端末にしない
  // O_SYNC: 書き込みが完了するまでwriteがブロックする (無くても良い場合が多い)
  int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
  {
    fprintf(stderr, "Error opening %s: %s\n", portname, strerror(errno));
    return -1;
  }

  // シリアルポートの属性設定
  if (set_interface_attribs(fd, speed) < 0)
  {
    close(fd);
    return -1;
  }

  tcflush(fd, TCIOFLUSH); // ポートの送受信バッファをクリア
  printf("Serial port %s opened and configured successfully.\n", portname);
  return fd;
}

// シリアルポート属性設定関数 (ノンブロッキング用)
int set_interface_attribs(int fd, int speed)
{
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) < 0)
  {
    perror("Error from tcgetattr");
    return -1;
  }

  // ボーレート設定
  cfsetospeed(&tty, (speed_t)speed);
  cfsetispeed(&tty, (speed_t)speed);

  // 制御フラグ設定
  tty.c_cflag |= (CLOCAL | CREAD); // ローカル接続、受信有効
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;      // 8データビット
  tty.c_cflag &= ~PARENB;  // パリティなし
  tty.c_cflag &= ~CSTOPB;  // ストップビット1
  tty.c_cflag &= ~CRTSCTS; // ハードウェアフロー制御なし

  // 入力フラグ設定 (Raw Input)
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // ソフトウェアフロー制御無効
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // 特殊文字処理無効

  // ローカルフラグ設定 (Raw Input)
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // カノニカルモード、エコー、シグナル文字無効

  // 出力フラグ設定 (Raw Output)
  tty.c_oflag &= ~OPOST;

  // パケット長単位でブロック: VMIN=5, VTIME=5(0.5s)
  tty.c_cc[VMIN] = SCAN_DATA_PACKET_SIZE;
  tty.c_cc[VTIME] = 5;

  // 設定を適用
  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    perror("Error from tcsetattr");
    return -1;
  }
  return 0;
}

// コマンド送信関数
int send_command(int fd, uint8_t cmd)
{
  uint8_t buffer[RPLIDAR_CMD_LEN] = {RPLIDAR_CMD_SYNC_BYTE, cmd};
  ssize_t written = write(fd, buffer, RPLIDAR_CMD_LEN);

  if (written < 0)
  {
    perror("Error writing command");
    return -1;
  }
  else if (written != (ssize_t)RPLIDAR_CMD_LEN)
  {
    fprintf(stderr, "Error: Incomplete command write (%zd/%d bytes)\n", written, RPLIDAR_CMD_LEN);
    return -1;
  }

  // 送信バッファの内容が物理的に送信されるのを待つ
  if (tcdrain(fd) == -1)
  {
    perror("Error waiting for transmission to complete (tcdrain)");
    // エラーだが、コマンド自体は送信された可能性が高い
    // return -1; // 続行不能とするか、警告に留めるか
  }

  usleep(WAIT_AFTER_CMD_US); // RPLiDARがコマンドを処理するための短い待機
  return 0;
}

// 指定時間内で指定バイト数を読み取る関数 (タイムアウト付き)
ssize_t read_with_timeout(int fd, uint8_t *buf, size_t n, useconds_t timeout_us)
{
  size_t total_read = 0;
  struct timeval start_time, current_time, elapsed_time;
  fd_set read_fds;
  struct timeval timeout_tv;

  gettimeofday(&start_time, NULL); // 開始時刻を記録

  while (total_read < n)
  {
    // タイムアウト値の計算
    gettimeofday(&current_time, NULL);
    timersub(&current_time, &start_time, &elapsed_time); // 経過時間
    useconds_t elapsed_us = elapsed_time.tv_sec * 1000000 + elapsed_time.tv_usec;

    if (elapsed_us >= timeout_us)
    {
      errno = ETIME;                                    // タイムアウトを示すerrno (独自)
      return total_read > 0 ? (ssize_t)total_read : -2; // タイムアウト (-2を返す)
    }

    useconds_t remaining_us = timeout_us - elapsed_us;
    timeout_tv.tv_sec = remaining_us / 1000000;
    timeout_tv.tv_usec = remaining_us % 1000000;

    // selectで読み取り可能になるまで待機 (タイムアウト付き)
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    int ret = select(fd + 1, &read_fds, NULL, NULL, &timeout_tv);

    if (ret == -1)
    {
      perror("select error in read_with_timeout");
      return -1; // selectエラー
    }
    else if (ret == 0)
    {
      errno = ETIME;                                    // タイムアウトを示すerrno (独自)
      return total_read > 0 ? (ssize_t)total_read : -2; // タイムアウト (-2を返す)
    }
    else
    { // ret > 0: データが読み取り可能
      ssize_t bytes_read = read(fd, buf + total_read, n - total_read);
      if (bytes_read > 0)
      {
        total_read += bytes_read;
      }
      else if (bytes_read == 0)
      {
        // 通常、シリアルポートではEOFは発生しにくいが念のため
        fprintf(stderr, "Warning: Read 0 bytes (EOF?)\n");
        usleep(1000); // 少し待機
      }
      else
      { // bytes_read < 0
        // EAGAIN/EWOULDBLOCK は select で待っているので通常発生しないはずだが念のため
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
          perror("Error reading in read_with_timeout");
          return -1; // 読み取りエラー
        }
        // EAGAIN/EWOULDBLOCK の場合はループを継続
        usleep(1000); // 少し待機
      }
    }
  } // end while

  return total_read; // 成功 (nバイト読み取った)
}