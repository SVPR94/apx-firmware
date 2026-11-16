# Calibration Guide

Accurate sensor calibration is vital for stable flight.

## Prerequisites
- Drone fully assembled.
- APX flashed and connected to APX Flight Manager via USB or Telemetry.

## Accelerometer Calibration
1. Open APX Flight Manager -> Vehicle Setup -> Sensors -> Accelerometer.
2. Click **Calibrate**.
3. Place the vehicle in the 6 standard orientations as prompted by the UI (Level, Left, Right, Nose Down, Nose Up, Upside Down).
4. Hold the vehicle completely still for a few seconds in each position.
- **Expected Time**: 2 minutes.

## Gyroscope Calibration
1. Select **Gyroscope**.
2. Click **Calibrate**.
3. Keep the vehicle perfectly still on a level surface until the progress bar completes.
- **Expected Time**: 30 seconds.

## Compass (Magnetometer) Calibration
1. Select **Compass**.
2. Click **Calibrate**.
3. Rotate the vehicle randomly around all axes. Try to cover a full sphere of orientations.
- **Expected Time**: 1-2 minutes.

## Troubleshooting

- **"High variance" error**: Ensure the vehicle is not vibrating or moving during gyro/accel calibration. Keep it on a solid table.
- **Compass calibration fails**: Move away from large metal objects (cars, reinforced concrete floors) and strong magnetic fields.
