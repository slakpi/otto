ENABLING UART FOR GPS
=====================

raspi-config Interfacing Options
--------------------------------

  * Disable login shell over Serial
  * Enable Serial

/boot/config.txt
----------------

  * Ensure: `enable_uart=1`
  * Add: `dtoverlay=pi3-miniuart-bt`
  * Add: `dtoverlay=pi3-disable-bt-overlay`

Verification
------------

  * Reboot
  * Verify: `/dev/serial0 -> /dev/ttyAMA0`