#!/bin/bash
# 树莓派（Ubuntu）网络加固：有线直连固定 IP + 关闭 WiFi 省电 + 开启 mDNS
# 用法：sudo bash network.sh
# 执行前先看一下现有配置：ls /etc/netplan/ ; nmcli con show
set -e

ETH_IP="192.168.1.101/24"     # 树莓派网口固定 IP（电脑网口设成 192.168.1.100，不填网关）

if systemctl is-active --quiet NetworkManager; then
    echo "== 使用 NetworkManager（Ubuntu 桌面版）"
    nmcli con delete robot-direct 2>/dev/null || true
    nmcli con add type ethernet ifname eth0 con-name robot-direct \
        ipv4.method manual ipv4.addresses "$ETH_IP" ipv4.never-default yes \
        ipv6.method disabled connection.autoconnect yes
    mkdir -p /etc/NetworkManager/conf.d
    printf '[connection]\nwifi.powersave = 2\n' > /etc/NetworkManager/conf.d/wifi-powersave-off.conf
    systemctl restart NetworkManager
else
    echo "== 使用 netplan（Ubuntu Server）"
    cat > /etc/netplan/60-robot-direct.yaml <<EOF
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: false
      addresses: [$ETH_IP]
      optional: true
EOF
    chmod 600 /etc/netplan/60-robot-direct.yaml
    netplan apply
    # WiFi 省电模式是 SSH 经常断的常见原因
    cat > /etc/systemd/system/wifi-powersave-off.service <<'EOF'
[Unit]
Description=Disable WiFi power save
After=network-online.target
[Service]
Type=oneshot
ExecStart=/usr/sbin/iw dev wlan0 set power_save off
[Install]
WantedBy=multi-user.target
EOF
    systemctl daemon-reload
    systemctl enable --now wifi-powersave-off.service || true
fi

# mDNS：以后可以直接 ssh ycy@<主机名>.local，不用记 IP
apt-get install -y avahi-daemon >/dev/null && systemctl enable --now avahi-daemon

# 串口权限
usermod -aG dialout ycy

echo "== 完成。主机名：$(hostname)   网口：$ETH_IP"
ip -4 -br addr
