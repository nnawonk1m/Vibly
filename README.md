# Vibly

Assistive wearable that classifies environmental sounds and renders them as
**localised haptics**, with its companion mobile app, firmware, and test tooling.

This repository is organised as **one branch per component** — `main` is an
index only. Each component lives on its own branch:

| Branch | What it holds |
|--------|----------------|
| [`software_application`](../../tree/software_application) | Vibly mobile app (Expo / React Native) that connects the device to the user's phone |
| [`mic_testing`](../../tree/mic_testing) | Dual-microphone debug/test scripts, model weights, and recorded audio data |
| [`vibly_mcu`](../../tree/vibly_mcu) | Main device firmware — audio classifier + localised haptics |
| [`ovd_imu`](../../tree/ovd_imu) | IMU firmware |

These branches hold unrelated components, so they have **independent histories**
and are not meant to be merged into one another.
