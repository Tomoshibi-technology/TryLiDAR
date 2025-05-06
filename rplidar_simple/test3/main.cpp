#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>   // ファイル制御 (open)
#include <termios.h> // POSIX ターミナル制御
#include <unistd.h>  // UNIX 標準関数 (write, close, usleep, read)
#include <errno.h>   // エラー番号
#include <math.h>    // 角度計算用 (fmodf)

// --- 定数定義 ---
// RPLiDAR コマンド
const uint8_t RPLIDAR_CMD_STOP = 0x25;
const uint8_t RPLIDAR_CMD_EXPRESS_SCAN = 0x82; // Express Scan コマンド
const uint8_t RPLIDAR_CMD_SYNC_BYTE = 0xA5;

// タイミング関連 (マイクロ秒)
const useconds_t WAIT_AFTER_CMD_US = 2000;       // コマンド送信後の待機
const useconds_t WAIT_AFTER_STOP_US = 10000;     // STOPコマンド後の待機

// Express Scan ペイロード
uint8_t payload[5] = {
    0x00, // working_mode = Legacy
    0x00, // Reserved
    0x00, // Reserved
    0x00, // Reserved
    0x00  // Reserved
};

// --- 関数プロトタイプ宣言 ---
int setup_serial_port(const char *portname, int speed);
int set_interface_attribs(int fd, int speed);
int send_command(int fd, uint8_t cmd, const void *payload = nullptr, size_t payloadsize = 0);

// Express Scan "Ultra Cabin" データパケット (84 bytes)
typedef struct
{
    uint8_t sync1;                 // 0xA5
    uint8_t sync2;                 // 0x5A
    uint8_t checksum_angle_q6_lsb; // Checksum [7], Start Angle Q6 [6:0]
    uint8_t start_angle_q6_msb;    // Start Angle Q6 [14:7]
    uint8_t cabin[80];             // 16 cabins * 5 bytes
} __attribute__((packed)) rplidar_response_ultra_cabin_t;

// データ関連定数
const size_t ULTRA_CABIN_SIZE = sizeof(rplidar_response_ultra_cabin_t); // 84

int main()
{
    const char *portname = "/dev/ttyAMA1";
    int fd = -1;

    // --- シリアルポート設定 ---
    fd = setup_serial_port(portname, B115200);
    if (fd < 0)
    {
        return -1;
    }

    // --- LiDAR制御 ---
    printf("Sending STOP command...\n");
    if (send_command(fd, RPLIDAR_CMD_STOP) < 0)
    {
        close(fd);
        return -1;
    }
    usleep(WAIT_AFTER_STOP_US);
    tcflush(fd, TCIOFLUSH); // バッファクリア
    printf("STOP command sent.\n");

    // --- Express Scan 開始 ---
    printf("Sending EXPRESS SCAN command (Mode 0x82)...\n");
    uint8_t checksum = 0xA5 ^ 0x82 ^ 0x05;
    for (int i = 0; i < 5; ++i)
    {
        checksum ^= payload[i];
    }
    uint8_t req[1 + 5 + 1]; // Header(2) + Size(1) + Payload(5) + Checksum(1)
    req[0] = 0x05;              // Payload Size
    memcpy(req + 1, payload, 5);
    req[6] = checksum;

    if (send_command(fd, RPLIDAR_CMD_EXPRESS_SCAN, req, sizeof(req)) < 0)
    {
        close(fd);
        return -1;
    }
    printf("EXPRESS SCAN command sent. LiDAR should start scanning.\n");

    // --- データ受信と表示 ---
    printf("Starting data reception loop (Press Ctrl+C to stop)...\n");
    rplidar_response_ultra_cabin_t cabin_data;
    uint8_t read_buffer[ULTRA_CABIN_SIZE]; // 一時的な読み込みバッファ
    size_t bytes_read_total = 0;
    uint8_t sync_state = 0; // 0: 初期状態, 1: Sync1(0xA5)受信済み

    while (1)
    { // 無限ループ (Ctrl+Cで停止)
        uint8_t current_byte;
        ssize_t n = read(fd, &current_byte, 1);

        if (n < 0)
        {
            if (errno == EINTR)
                continue; // シグナル割り込みの場合は継続
            perror("Error reading from serial port");
            break; // その他のエラー発生時はループ終了
        }
        else if (n == 0)
        {
            // タイムアウトなし設定なので通常ここには来ないはずだが、念のため
            usleep(1000); // 少し待機
            continue;
        }

        // --- 同期バイト (0xA5, 0x5A) を探す ---
        if (sync_state == 0)
        {
            if (current_byte == RPLIDAR_CMD_SYNC_BYTE)
            { // 0xA5
                sync_state = 1;
                read_buffer[0] = current_byte;
                bytes_read_total = 1;
            }
        }
        else if (sync_state == 1)
        {
            if (current_byte == 0x5A)
            { // Sync2 受信
                read_buffer[1] = current_byte;
                bytes_read_total = 2;

                // --- Ultra Cabin の残りデータを読み込む ---
                while (bytes_read_total < ULTRA_CABIN_SIZE)
                {
                    ssize_t remaining_bytes = ULTRA_CABIN_SIZE - bytes_read_total;
                    n = read(fd, &read_buffer[bytes_read_total], remaining_bytes);
                    if (n < 0)
                    {
                        if (errno == EINTR)
                            continue; // シグナル割り込み
                        perror("Error reading remaining packet data");
                        sync_state = 0; // エラー時は同期状態リセット
                        goto next_byte; // ループの先頭に戻る (gotoは推奨されないが、ここでは簡潔さのため使用)
                    }
                    else if (n == 0)
                    {
                        usleep(1000); // 待機
                        fprintf(stderr, "Warning: Incomplete packet received.\n");
                        sync_state = 0; // 不完全なパケットは破棄して同期リセット
                        goto next_byte;
                    }
                    bytes_read_total += n;
                }

                // --- データが揃ったら構造体にコピーして処理 ---
                memcpy(&cabin_data, read_buffer, ULTRA_CABIN_SIZE);

                // --- チェックサム検証 (省略) ---
                // 必要であればここにチェックサム計算と検証処理を追加

                // --- データの解析と表示 ---
                float start_angle_deg = ((cabin_data.start_angle_q6_msb << 7) | (cabin_data.checksum_angle_q6_lsb & 0x7F)) / 64.0f;

                for (int i = 0; i < 16; ++i)
                { // 16 cabins per Ultra Cabin
                    uint8_t *cabin_ptr = cabin_data.cabin + i * 5;
                    // 各キャビンには2つのサンプルが含まれる
                    uint16_t dist1_mm_q2 = (cabin_ptr[1] << 8) | cabin_ptr[0];
                    uint16_t dist2_mm_q2 = (cabin_ptr[3] << 8) | cabin_ptr[2];
                    uint8_t delta_angles_q3 = cabin_ptr[4];

                    // 角度差分を計算
                    float delta_angle1_deg = (delta_angles_q3 & 0x0F) / 8.0f;
                    float delta_angle2_deg = (delta_angles_q3 >> 4) / 8.0f;

                    // 1番目のサンプルの角度と距離
                    float angle1 = start_angle_deg + delta_angle1_deg;
                    float dist1_mm = dist1_mm_q2 / 4.0f;

                    // 角度を 0-360 の範囲に正規化
                    angle1 = fmodf(angle1, 360.0f);
                    if (angle1 < 0)
                        angle1 += 360.0f;

                    // 有効な距離データのみ表示 (距離0は無効データ)
                    if (dist1_mm > 0)
                    {
                        printf("Angle: %7.2f deg, Distance: %7.2f mm\n", angle1, dist1_mm);
                    }

                    // 2番目のサンプルの角度と距離 (角度の計算は次のキャビンとの関係で複雑になるため注意)
                    float dist2_mm = dist2_mm_q2 / 4.0f;

                    // 次のキャビンの開始角度を更新 (次のループイテレーション用)
                    start_angle_deg += delta_angle1_deg + delta_angle2_deg;
                }

                // 次の同期バイトを探すためにリセット
                sync_state = 0;
                bytes_read_total = 0;
            }
            else if (current_byte == RPLIDAR_CMD_SYNC_BYTE)
            { // 0xA5
                // Sync1 の後で再び Sync1 が来た場合、状態を維持してバッファをリセット
                read_buffer[0] = current_byte;
                bytes_read_total = 1;
            }
            else
            {
                // Sync1 の後で Sync2 (0x5A) 以外が来た場合はリセット
                sync_state = 0;
                bytes_read_total = 0;
            }
        }
    next_byte:; // goto ラベル (エラー処理用)
    } // end while(1)

    // --- 終了処理 ---
    // ループが終了した場合 (エラー発生時など)
    printf("\nStopping LiDAR and closing port...\n");
    send_command(fd, RPLIDAR_CMD_STOP); // STOPコマンド送信
    usleep(WAIT_AFTER_STOP_US);         // 少し待つ
    close(fd);
    printf("Serial port closed.\n");

    return 0; // 通常はここまで到達しない (Ctrl+Cで終了するため)
}

// --- 関数定義 ---

// シリアルポートのオープンと設定
int setup_serial_port(const char *portname, int speed)
{
    // O_RDWR: 読み書きモード
    // O_NOCTTY: このプロセスを制御端末にしない
    // O_SYNC: 書き込みが完了するまでwriteがブロックする
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        fprintf(stderr, "Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    if (set_interface_attribs(fd, speed) < 0)
    {
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    printf("Serial port %s opened and configured successfully.\n", portname);
    return fd;
}

// シリアルポート属性設定
int set_interface_attribs(int fd, int speed)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) < 0)
    {
        perror("Error from tcgetattr");
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    tty.c_oflag &= ~OPOST;

    // read() の動作設定 (ここでは最小1バイト、タイムアウトなしに設定)
    tty.c_cc[VMIN] = 1;  // 少なくとも1バイト読み込むまでブロック
    tty.c_cc[VTIME] = 0; // タイムアウトなし

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("Error from tcsetattr");
        return -1;
    }
    return 0;
}

// コマンド送信関数
int send_command(int fd, uint8_t cmd, const void *payload, size_t payloadsize)
{
    uint8_t buffer[2 + payloadsize]; // Sync + Cmd + Payload
    buffer[0] = RPLIDAR_CMD_SYNC_BYTE;
    buffer[1] = cmd;
    if (payload && payloadsize > 0)
    {
        memcpy(&buffer[2], payload, payloadsize);
    }
    size_t total_len = 2 + payloadsize;

    ssize_t written = write(fd, buffer, total_len);

    if (written < 0)
    {
        perror("Error writing command");
        return -1;
    }
    else if ((size_t)written != total_len)
    {
        fprintf(stderr, "Error: Incomplete command write (%zd/%zu bytes)\n", written, total_len);
        return -1;
    }

    if (tcdrain(fd) == -1)
    {
        perror("Error waiting for transmission to complete (tcdrain)");
        // return -1; // 続行不能とするか判断
    }

    usleep(WAIT_AFTER_CMD_US);
    return 0;
}