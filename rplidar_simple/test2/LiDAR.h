#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include <string>
#include <termios.h> // speed_t (B115200) のため

// --- 定数定義 ---
// データサイズ (クラス内で使用)
#define RPLIDAR_CMD_LEN 2
#define SCAN_RESPONSE_DESCRIPTOR_SIZE 7
#define SCAN_DATA_PACKET_SIZE 5

// --- データパケット構造体 ---
typedef struct
{
    uint8_t sync_quality;      // Sync bit (LSB=1), Inverse Sync bit (LSB+1=0), Quality (6 bits MSB)
    uint8_t angle_q6_checkbit; // Angle (lower 7 bits = LSB), Check bit (MSB=1) <- Check bitは旧仕様？ 通常は無視
    uint8_t angle_q14;         // Angle (upper 8 bits = MSB)
    uint8_t distance_q2_lsb;   // Distance (lower 8 bits)
    uint8_t distance_q2_msb;   // Distance (upper 8 bits)
} __attribute__((packed)) rplidar_response_measurement_node_t;

class LiDAR
{
public:
    // コンストラクタとデストラクタ
    LiDAR(const std::string &port, speed_t baudrate = B115200);
    ~LiDAR();

    // LiDAR制御メソッド
    bool start_scan();
    bool stop_scan();
    bool read_scan_data(float &angle_deg, float &distance_mm, uint8_t &quality);
    bool is_connected() const;

private:
    // プライベートメンバー変数
    int fd_;                     // ファイルディスクリプタ
    std::string portname_;       // シリアルポート名
    speed_t baudrate_;           // ボーレート
    bool is_scanning_;           // スキャン中フラグ
    rplidar_response_measurement_node_t current_node_; // 現在のデータノード

    // プライベートヘルパーメソッド
    bool setup_serial_port();
    bool set_interface_attribs();
    bool send_command(uint8_t cmd);
    ssize_t read_with_timeout(uint8_t *buf, size_t n, useconds_t timeout_us);
    bool read_packet(uint8_t *packet_buffer);
};

#endif // LIDAR_H
