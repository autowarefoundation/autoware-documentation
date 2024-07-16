# Enable `multicast` on `lo`

## Manually

You may just call the following command to enable multicast on the loopback interface.

```bash
sudo ip link set lo multicast on
```

!!! warning

    This will be reverted once the computer restarts. To make it permanent, follow the steps below.

## On startup with a service

```bash
sudo nano /etc/systemd/system/multicast-lo.service
```

Paste the following into the file:

```service
[Unit]
Description=Enable Multicast on Loopback

[Service]
Type=oneshot
ExecStart=/usr/sbin/ip link set lo multicast on

[Install]
WantedBy=multi-user.target
```

Press following in order to save with nano:

1. `Ctrl+X`
2. `Y`
3. `Enter`

```bash
# Make it recognized
sudo systemctl daemon-reload

# Make it run on startup
sudo systemctl enable multicast-lo.service

# Start it now
sudo systemctl start multicast-lo.service
```

### Validate

```console
you@pc:~$ sudo systemctl status multicast-lo.service
○ multicast-lo.service - Enable Multicast on Loopback
     Loaded: loaded (/etc/systemd/system/multicast-lo.service; enabled; vendor preset: enabled)
     Active: inactive (dead) since Mon 2024-07-08 12:54:17 +03; 4s ago
    Process: 22588 ExecStart=/usr/bin/ip link set lo multicast on (code=exited, status=0/SUCCESS)
   Main PID: 22588 (code=exited, status=0/SUCCESS)
        CPU: 1ms

Tem 08 12:54:17 mfc-leo systemd[1]: Starting Enable Multicast on Loopback...
Tem 08 12:54:17 mfc-leo systemd[1]: multicast-lo.service: Deactivated successfully.
Tem 08 12:54:17 mfc-leo systemd[1]: Finished Enable Multicast on Loopback.
```

```console
you@pc:~$ ip link show lo
1: lo: <LOOPBACK,MULTICAST,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
```

### Uninstalling the service

If for some reason you want to uninstall the service, you can do so by following these steps:

```bash
# Stop the service
sudo systemctl stop multicast-lo.service

# Disable the service from running on startup
sudo systemctl disable multicast-lo.service

# Remove the service file
sudo rm /etc/systemd/system/multicast-lo.service

# Reload systemd to apply the changes
sudo systemctl daemon-reload
```
