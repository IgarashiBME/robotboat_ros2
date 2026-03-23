# systemd 自動起動サービス

自律走行ローバーの5つの ROS2 ノードを systemd で起動時に自動実行する。

## ファイル構成

| ファイル | 役割 |
|------|------|
| `env.sh` | 共通環境設定: ROS2 + ワークスペースの source、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |
| `scripts/launch_*.sh` | 各ノードの起動スクリプト（5つ）。ublox と look_ahead は venv も activate。全て `exec` で PID を引き継ぎ |
| `systemd/ros2-*.service` | systemd ユニットファイル（5つ）。`User=kikai`、`Restart=on-failure`、`RestartSec=5` |
| `setup_services.sh` | インストーラ: 権限付与、シンボリックリンク、daemon-reload、enable、dialout 確認 |

## インストール方法

```bash
cd ~/ros2_ws/deploy && sudo bash setup_services.sh
```

## テスト方法

```bash
# 単体テスト
sudo systemctl start ros2-mavlink
journalctl -u ros2-mavlink -f

# 全サービス起動/停止
sudo systemctl start ros2-ublox ros2-ntrip ros2-mavlink ros2-look-ahead ros2-maestro
sudo systemctl stop ros2-ublox ros2-ntrip ros2-mavlink ros2-look-ahead ros2-maestro
```
