#include <cstdio>   // printf, fprintf, perror
#include <csignal>  // sigaction, sigemptyset, SIGINT
#include <cstring>  // memset
#include <unistd.h> // usleep (optional)
#include "LiDAR.h"

// --- グローバル変数 ---
volatile sig_atomic_t stop_flag = 0; // Ctrl+Cハンドラ用

// --- Ctrl+C ハンドラ ---
void handle_sigint(int sig)
{
    (void)sig; // 未使用引数警告を抑制
    stop_flag = 1;
    printf("\nCtrl+C detected, stopping...\n");
}

int main()
{
    const char *portname = "/dev/ttyAMA1"; // 環境に合わせて変更

    // --- Ctrl+C ハンドラ設定 ---
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_sigint;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0; // SA_RESTART は設定しない (readなどが中断されるように)
    if (sigaction(SIGINT, &sa, NULL) == -1)
    {
        perror("Error setting SIGINT handler");
        return -1;
    }
    printf("Ctrl+C handler set. Press Ctrl+C to stop.\n");

    // --- LiDAR オブジェクト生成 ---
    LiDAR lidar(portname); // デフォルトボーレート B115200 を使用

    // 接続確認
    if (!lidar.is_connected())
    {
        fprintf(stderr, "Failed to connect to LiDAR.\n");
        return -1;
    }

    // --- スキャン開始 ---
    if (!lidar.start_scan())
    {
        fprintf(stderr, "Failed to start LiDAR scan.\n");
        return -1;
    }

    printf("Starting to read scan data...\n");

    // --- スキャンデータ読み取りループ ---
    float angle = 0.0f;
    float distance = 0.0f;
    uint8_t quality = 0;

    while (!stop_flag)
    {
        if (lidar.read_scan_data(angle, distance, quality))
        {
            // 距離が0mmでない場合のみ出力
            if (distance != 0.0f)
            {
                // start_bit は現在取得していないため、常にスペースを出力
                printf("  Theta: %07.2f Deg, Dist: %08.2f mm, Q: %d\n",
                       angle, distance, quality);
            }
        }
        else
        {
            // read_scan_data が false を返した場合
            if (stop_flag) {
                // Ctrl+C が押された場合、ループを抜ける
                break;
            }
            // 読み取りエラーまたはタイムアウトが発生した可能性がある
            // 必要であればエラー処理を追加 (例: usleepで待機、再試行など)
            // fprintf(stderr, "Failed to read scan data packet.\n");
            usleep(1000); // 短い待機を入れてCPU負荷を軽減
        }
    }

    // --- 終了処理 ---
    // LiDARオブジェクトのデストラクタが stop_scan() と close() を呼び出す
    printf("Exiting program.\n");

    return 0;
}
