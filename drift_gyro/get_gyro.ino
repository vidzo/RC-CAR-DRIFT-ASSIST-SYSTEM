inline void callimu() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float rawGz = (gz - offsetGz) * 0.030516;
  yawRateFiltered = gyroFilterAlpha * rawGz + (1 - gyroFilterAlpha) * yawRateFiltered;
  G[2] = yawRateFiltered;
}
