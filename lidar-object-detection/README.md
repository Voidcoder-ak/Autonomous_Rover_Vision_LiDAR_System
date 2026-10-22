# lidar_stop

LiDAR step 1: stop the rover when something is in front of it, and measure free space on the left and right (the input the bypass feature needs later). Written for the **YDLIDAR X2**, in C++, using the official YDLidar SDK.

## 1. Plug in and check (5 min)

1. Connect the X2 to its small USB adapter board, then to the laptop. The motor should start spinning.
2. Check that Linux sees it:
   ```bash
   ls /dev/ttyUSB*        # expect /dev/ttyUSB0
   ```
3. Give yourself permission to use serial ports (once, then **log out and back in**):
   ```bash
   sudo usermod -aG dialout $USER
   ```

If the motor doesn't spin, it isn't getting 5 V. Try a different USB port or a powered hub. Some X2 adapter boards also have a second USB port that's only for power.

## 2. Install the YDLidar SDK (once)

```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK && mkdir build && cd build
cmake .. && make -j4 && sudo make install
```

Quick test with the SDK's own example: run `tri_test`, pick the port, choose baud `[0] 115200`, and answer **yes** to "one-way communication". You should see `Scan received [4xx] points`.

## 3. Build and run

```bash
cd lidar_stop && mkdir build && cd build && cmake .. && make
./lidar_stop --sim                      # no LiDAR needed: fake room + moving box
./lidar_stop --port /dev/ttyUSB0        # the real X2, radar window
```

Press `q`, `Esc` or Ctrl+C to quit (the LiDAR motor stops cleanly).

## 4. Calibrate "forward" (2 min, required)

The X2's 0° mark may not point where your rover's front is.

1. Run `./lidar_stop --port /dev/ttyUSB0 --calib`.
2. Hold your hand about 20 cm in **front** of the rover. It prints e.g. `nearest point: 0.21 m at +90 deg`.
3. Run with the opposite of that angle: `--offset -90`. Your hand should now appear straight **up** in the radar window.
4. Move your hand to the rover's **left**. If it appears on the right of the window, add `--mirror`.

Then put your final `--offset` value into `forwardOffsetDeg` in `Config`.

## 5. Measure your rover (in `Config`)

| Setting | What to measure |
|---|---|
| `lidarToFrontM` | LiDAR centre to the front bumper |
| `lidarToRearM` | LiDAR centre to the rear edge |
| `halfWidthM` | Half the rover's width |
| `stopDistM` / `resumeDistM` | Stop at 30 cm, resume at 40 cm (keep the gap) |

## Known limits

- **Height:** the X2 scans one flat plane at its own height. Objects lower than the LiDAR (a shoe, a cable) are invisible to it. The camera covers those.
- **Glass and mirrors:** the laser passes through or bounces off them.
- **Very dark or shiny surfaces** can return few points.
- **Thin objects** (chair legs) return only 1–2 points. The filter keeps anything with 2 or more points, so a single stray point is ignored.
