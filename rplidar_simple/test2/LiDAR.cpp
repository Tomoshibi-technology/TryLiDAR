#include "LiDAR.h"
#include <fcntl.h>    // open
#include <unistd.h>   // read, write, close, usleep
#include <termios.h>  // tcgetattr, tcsetattr, cfsetospeed, cfsetispeed, tcflush
#include <string.h>   // strerror, memset, memcpy
#include <errno.h>    // errno
#include <sys/time.h> // gettimeofday, timersub, select
#include <stdio.h>    // fprintf, perror, printf (デバッグ用)
#include <stdlib.h>   // exit (エラー時)

// --- 定数定義 (LiDAR.cpp 内でのみ使用) ---
const uint8_t RPLIDAR_CMD_STOP = 0x25;
const uint8_t RPLIDAR_CMD_SCAN = 0x20;
// const uint8_t RPLIDAR_CMD_FORCE_SCAN = 0x21; // 通常のSCANを推奨
// const uint8_t RPLIDAR_CMD_RESET = 0x40;
// const uint8_t RPLIDAR_CMD_GET_INFO = 0x50;
// const uint8_t RPLIDAR_CMD_GET_HEALTH = 0x52;
const uint8_t RPLIDAR_CMD_SYNC_BYTE = 0xA5;

// タイミング関連 (マイクロ秒)
const useconds_t WAIT_AFTER_CMD_US = 2000;      // コマンド送信後の待機
const useconds_t WAIT_AFTER_STOP_US = 10000;    // STOPコマンド後の待機
const useconds_t WAIT_MOTOR_STABLE_US = 200000; // モーター安定待ち (必要に応じて調整)
const useconds_t READ_TIMEOUT_US = 1000000;     // 応答デスクリプタ読み取りタイムアウト (1秒)
// const useconds_t READ_LOOP_WAIT_US = 1000;      // データ読み取りループでの待機 (CPU負荷軽減) - read_packet内で処理

// SCAN応答デスクリプタの期待値
const uint8_t EXPECTED_SCAN_DESCRIPTOR[SCAN_RESPONSE_DESCRIPTOR_SIZE] = {
    RPLIDAR_CMD_SYNC_BYTE, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};

// --- コンストラクタ ---
LiDAR::LiDAR(const std::string &port, speed_t baudrate)
    : fd_(-1), portname_(port), baudrate_(baudrate), is_scanning_(false)
{
    if (!setup_serial_port())
    {
        fprintf(stderr, "Failed to initialize LiDAR on port %s\n", portname_.c_str());
        // fd_ は setup_serial_port 内で -1 のままのはず
    }
}

// --- デストラクタ ---
LiDAR::~LiDAR()
{
    if (is_scanning_)
    {
        stop_scan();
    }
    if (fd_ >= 0)
    {
        close(fd_);
        printf("Serial port %s closed.\n", portname_.c_str());
    }
}

// --- public メソッド ---

bool LiDAR::is_connected() const
{
    return fd_ >= 0;
}

bool LiDAR::start_scan()
{
    if (fd_ < 0)
    {
        fprintf(stderr, "Error: Serial port not open.\n");
        return false;
    }
    if (is_scanning_)
    {
        fprintf(stderr, "Warning: Already scanning.\n");
        return true; // すでにスキャン中なら成功扱い
    }

    // 既存のスキャンを停止 (念のため)
    printf("Sending STOP command (before starting scan)...\n");
    if (!send_command(RPLIDAR_CMD_STOP))
    {
        // エラーでも続行を試みる
        fprintf(stderr, "Warning: Failed to send initial STOP command.\n");
    }
    usleep(WAIT_AFTER_STOP_US);
    tcflush(fd_, TCIOFLUSH); // バッファクリア

    // モーター安定待ち
    printf("Waiting for motor to stabilize...\n");
    usleep(WAIT_MOTOR_STABLE_US);

    // スキャン開始コマンド送信
    printf("Sending SCAN command...\n");
    if (!send_command(RPLIDAR_CMD_SCAN))
    {
        fprintf(stderr, "Error: Failed to send SCAN command.\n");
        return false;
    }

    // 応答デスクリプタ読み取り & 検証
    uint8_t descriptor_buf[SCAN_RESPONSE_DESCRIPTOR_SIZE];
    printf("Reading SCAN response descriptor...\n");
    ssize_t desc_read = read_with_timeout(descriptor_buf, SCAN_RESPONSE_DESCRIPTOR_SIZE, READ_TIMEOUT_US);

    if (desc_read == (ssize_t)SCAN_RESPONSE_DESCRIPTOR_SIZE)
    {
        if (memcmp(descriptor_buf, EXPECTED_SCAN_DESCRIPTOR, SCAN_RESPONSE_DESCRIPTOR_SIZE) == 0)
        {
            printf("Scan response descriptor OK.\n");
            is_scanning_ = true;
            return true;
        }
        else
        {
            fprintf(stderr, "Error: Invalid scan response descriptor received: ");
            for (size_t i = 0; i < SCAN_RESPONSE_DESCRIPTOR_SIZE; ++i)
            {
                fprintf(stderr, "0x%02X ", descriptor_buf[i]);
            }
            fprintf(stderr, "\n");
            send_command(RPLIDAR_CMD_STOP); // 念のためSTOP送信
            return false;
        }
    }
    else
    {
        fprintf(stderr, "Error reading scan response descriptor (read %zd bytes, expected %zu)\n",
                desc_read, SCAN_RESPONSE_DESCRIPTOR_SIZE);
        if (desc_read == -2) { // read_with_timeout でのタイムアウト
             fprintf(stderr, " (Timeout)\n");
        } else if (desc_read < 0) {
             perror(" Read error");
        }
        send_command(RPLIDAR_CMD_STOP); // 念のためSTOP送信
        return false;
    }
}

bool LiDAR::stop_scan()
{
    if (fd_ < 0)
    {
        // ポートが開いていない場合は何もしない
        return true;
    }
    if (!is_scanning_)
    {
        // スキャン中でなければSTOPを送る必要はないかもしれないが、念のため送る
        printf("Sending STOP command (explicitly called)...\n");
    } else {
        printf("Stopping scan...\n");
    }


    bool success = send_command(RPLIDAR_CMD_STOP);
    if (!success)
    {
        fprintf(stderr, "Warning: Failed to send STOP command.\n");
    }
    usleep(WAIT_AFTER_STOP_US);
    tcflush(fd_, TCIOFLUSH); // バッファクリア
    is_scanning_ = false;
    return success; // コマンド送信の成否を返す
}

bool LiDAR::read_scan_data(float &angle_deg, float &distance_mm, uint8_t &quality)
{
    if (!is_scanning_)
    {
        // fprintf(stderr, "Error: Not scanning.\n"); // 頻繁に出力される可能性があるのでコメントアウト
        return false;
    }

    uint8_t packet_buffer[SCAN_DATA_PACKET_SIZE];
    if (!read_packet(packet_buffer))
    {
        // read_packet内でエラーメッセージ表示済みのはず
        return false;
    }

    // パケットを構造体にコピー
    memcpy(&current_node_, packet_buffer, SCAN_DATA_PACKET_SIZE);

    // データ抽出と計算
    quality = current_node_.sync_quality >> 2;
    // uint8_t start_bit = current_node_.sync_quality & 0x01; // 必要なら利用
    uint16_t angle_raw = ((uint16_t)current_node_.angle_q14 << 7) | (current_node_.angle_q6_checkbit & 0x7F);
    angle_deg = angle_raw / 64.0f;
    uint16_t dist_raw = ((uint16_t)current_node_.distance_q2_msb << 8) | current_node_.distance_q2_lsb;
    distance_mm = dist_raw / 4.0f;

    return true; // 成功
}

// --- private ヘルパーメソッド ---

bool LiDAR::setup_serial_port()
{
    // O_RDWR: 読み書きモード
    // O_NOCTTY: このプロセスを制御端末にしない
    // O_SYNC: 書き込みが完了するまでwriteがブロックする (無くても良い場合が多い)
    // O_NONBLOCK: readがブロックしないようにする (read_with_timeoutでselectを使うため)
    fd_ = open(portname_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
        fprintf(stderr, "Error opening %s: %s\n", portname_.c_str(), strerror(errno));
        return false;
    }

    // シリアルポートの属性設定
    if (!set_interface_attribs())
    {
        close(fd_);
        fd_ = -1;
        return false;
    }

    tcflush(fd_, TCIOFLUSH); // ポートの送受信バッファをクリア
    printf("Serial port %s opened and configured successfully.\n", portname_.c_str());
    return true;
}

bool LiDAR::set_interface_attribs()
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd_, &tty) < 0)
    {
        perror("Error from tcgetattr");
        return false;
    }

    // ボーレート設定
    cfsetospeed(&tty, baudrate_);
    cfsetispeed(&tty, baudrate_);

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

    // 読み取り制御 (VMIN=0, VTIME=0): ノンブロッキング read
    // read_with_timeout で select を使うため、ここではブロックしない設定にする
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    // 設定を適用
    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        perror("Error from tcsetattr");
        return false;
    }
    return true;
}

bool LiDAR::send_command(uint8_t cmd)
{
    if (fd_ < 0) return false;

    uint8_t buffer[RPLIDAR_CMD_LEN] = {RPLIDAR_CMD_SYNC_BYTE, cmd};
    ssize_t written = write(fd_, buffer, RPLIDAR_CMD_LEN);

    if (written < 0)
    {
        perror("Error writing command");
        return false;
    }
    else if (written != (ssize_t)RPLIDAR_CMD_LEN)
    {
        fprintf(stderr, "Error: Incomplete command write (%zd/%d bytes)\n", written, RPLIDAR_CMD_LEN);
        return false;
    }

    // 送信バッファの内容が物理的に送信されるのを待つ
    if (tcdrain(fd_) == -1)
    {
        perror("Error waiting for transmission to complete (tcdrain)");
        // エラーだが、コマンド自体は送信された可能性が高いので true を返す
    }

    usleep(WAIT_AFTER_CMD_US); // RPLiDARがコマンドを処理するための短い待機
    return true;
}

// 指定時間内で指定バイト数を読み取る関数 (タイムアウト付き)
// 戻り値: 読み取ったバイト数、タイムアウト時は -2、エラー時は -1
ssize_t LiDAR::read_with_timeout(uint8_t *buf, size_t n, useconds_t timeout_us)
{
    if (fd_ < 0) return -1;

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
            // errno = ETIME; // errnoを直接設定するのは避ける
            return total_read > 0 ? (ssize_t)total_read : -2; // タイムアウト (-2を返す)
        }

        useconds_t remaining_us = timeout_us - elapsed_us;
        timeout_tv.tv_sec = remaining_us / 1000000;
        timeout_tv.tv_usec = remaining_us % 1000000;

        // selectで読み取り可能になるまで待機 (タイムアウト付き)
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        int ret = select(fd_ + 1, &read_fds, NULL, NULL, &timeout_tv);

        if (ret == -1)
        {
            // EINTR (シグナルによる中断) はリトライする
            if (errno == EINTR) continue;
            perror("select error in read_with_timeout");
            return -1; // selectエラー
        }
        else if (ret == 0)
        {
            // errno = ETIME;
            return total_read > 0 ? (ssize_t)total_read : -2; // タイムアウト (-2を返す)
        }
        else
        { // ret > 0: データが読み取り可能
            ssize_t bytes_read = read(fd_, buf + total_read, n - total_read);
            if (bytes_read > 0)
            {
                total_read += bytes_read;
            }
            else if (bytes_read == 0)
            {
                // ポートが閉じられた？ 通常シリアルでは稀
                fprintf(stderr, "Warning: Read 0 bytes (EOF?)\n");
                usleep(1000); // 少し待機
            }
            else
            { // bytes_read < 0
                // EINTR (シグナルによる中断) はリトライする
                if (errno == EINTR) continue;
                // EAGAIN/EWOULDBLOCK はノンブロッキングreadでデータがない場合。selectを通しているので通常発生しないはずだが、念のためリトライ
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    usleep(1000); // 少し待機してリトライ
                    continue;
                }
                perror("Error reading in read_with_timeout");
                return -1; // その他の読み取りエラー
            }
        }
    } // end while

    return total_read; // 成功 (nバイト読み取った)
}

// 1パケット (SCAN_DATA_PACKET_SIZE バイト) を読み取るヘルパー関数
// タイムアウトは内部で設定 (例: 0.5秒)
bool LiDAR::read_packet(uint8_t *packet_buffer) {
    const useconds_t packet_timeout_us = 500000; // 0.5秒
    ssize_t rd = read_with_timeout(packet_buffer, SCAN_DATA_PACKET_SIZE, packet_timeout_us);

    if (rd == (ssize_t)SCAN_DATA_PACKET_SIZE) {
        return true; // 成功
    } else if (rd == -2) {
        // fprintf(stderr, "Timeout reading scan data packet.\n"); // 頻繁に出るのでコメントアウト
        return false;
    } else if (rd < 0) {
        fprintf(stderr, "Error reading scan data packet (read returned %zd).\n", rd);
        return false;
    } else { // rd >= 0 && rd < SCAN_DATA_PACKET_SIZE
        fprintf(stderr, "Incomplete scan data packet read (%zd/%d bytes).\n", rd, SCAN_DATA_PACKET_SIZE);
        return false;
    }
}
