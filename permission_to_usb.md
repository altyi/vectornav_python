# Grant non-root permission to ttyUSB

1. To find out which user group the device is attached to:
    ```
    stat /dev/ttyUSB0
    ```
2. This should produce something like
    ```
    Gid: (   20/ dialout)
    ```
3. Just add your user to the dialout group so you have appropriate permissions on the device.
    ```
    sudo usermod -a -G dialout $USER
    ```