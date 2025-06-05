
# Capstone Project: Developing a Sound Source Localization System For Teleconference Applications

This project integrates real-time audio signal processing on STM32 microcontrollers with source localization algorithms using a 4-microphone array and camera rotation toward the speaker. 
For the demonstration video, check out this Youtube link: https://www.youtube.com/watch?v=IOeTKqY6mDM

## Repository Structure

```
├── Matlab Simulation and Audio Analysis/
│   ├── dataset/                  # Sample WAV files and MATLAB test signals
│   ├── beamforming.m             # Delay-and-sum and MUSIC simulations
│   ├── plot_results.m            # Visualization of direction-of-arrival estimation
│   └── ...                       # Additional MATLAB analysis scripts
│
├── data recording/
│   ├── stm32_audio_recorder/    # STM32 firmware to record from 8 microphones
│   ├── write_wav.c              # FATFS-compatible WAV writer for SD card
│   ├── config.h                 # Microphone array and audio format settings
│   └── ...                      # Peripheral configs for SAI/I2S and DMA
│
├── main algorithm/
│   ├── vad.h / beamforming.h    # Modular header files for processing pipeline
│   ├── gccphat.h / music.h      # GCC-PHAT and MUSIC algorithm implementations
│   ├── fft_utils.c              # CMSIS-DSP FFT wrappers
│   └── main.c                   # Main processing loop (FFT, VAD, DOA estimation)
```

## Project Goals

- **Multichannel Audio Capture**: Record synchronized audio from 4 digital MEMS microphones (via I2S).
- **Real-Time DSP**: Implement FFT, VAD (Voice Activity Detection), and noise suppression on STM32.
- **Direction of Arrival Estimation**:
  - Delay-and-Sum Beamforming
  - GCC-PHAT
  - MUSIC Algorithm (SVD-based, using Jacobi one-sided)
- **Rotate the camera toward the speaker
- **MATLAB Analysis**: Simulate localization algorithms and validate embedded results using real recordings.

## Technologies Used

- **STM32F4/H7** with CubeMX, HAL drivers
- **CMSIS-DSP** for efficient FFT and vector math
- **FatFs** for SD card storage
- **MATLAB** for offline testing, plotting, and simulation

## Getting Started

1. **Embedded Setup**:
   - Flash the code from `data recording/` and `main algorithm/` onto STM32.
   - Connect 4 I2S/SAI digital MEMS mics in a 0.06m square geometry.
   - Log frequency/angle data via UART or save WAVs to SD card.

2. **Simulation**:
   - Use the `.wav` files in `Matlab Simulation and Audio Analysis/dataset/`.
   - Run `beamforming.m` or `music.m` to test and visualize localization accuracy.

## Output Example

The system logs dominant frequencies and DOA angles like:
```
Mic 1: 512 Hz  |  Angle: 45°
Mic 2: 512 Hz  |  Angle: 47°
...
```

In MATLAB, polar plots show signal peaks around the estimated direction.

## 🙋 Authors & Credits

Developed by Nguyen Thi Quynh Anh, Fulbright University, 2025  
Contributor: `@crazyblueer`

## 📜 License

MIT License
