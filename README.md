# AI-Powered Edge Gunshot Detection for Wildlife Protection

## Short summary
This repository contains firmware, hardware integration code, and supporting artifacts for an edge device that detects gunshot-like events using a Raspberry Pi Pico. The device performs on-device audio capture and inference, attaches GPS coordinates, and reports events over an NRF24L01 wireless link. The primary goals are real-time detection, low power operation, and field deployability for anti-poaching monitoring.

## Status
- Development: Feature-complete for core sensing, radio communication, GPS and power management on the Pico hardware.
- Machine learning: A TensorFlow Lite quantized model and export artifacts are provided; on-device inference integration is in progress.

Key completed items
- Audio input: INMP441 I2S microphone with PIO/DMA sampling.
- Location: NEO-6M GPS integration and basic NMEA parsing.
- Radio: NRF24L01 packet transmission with fragmentation and retries.
- Power: IP5306 power management using a PIO-driven PWM (continuous KEY pulsing) for reliable operation.
- Architecture: Dual-core Pico split (system/core tasks and audio/inference tasks) with robust initialization and error handling.

Planned / in development
- Finalize TensorFlow Lite inference on Pico and optimize model runtime.
- Optional mesh networking, encryption and acknowledgments.

## Repository contents (high level)
- `Raspberry_pi_pico/`, `inference-app/` – Pico firmware and build files.
- `python/` – training artifacts, quantized model (`gunshot_model_quant.tflite`), and helper notebooks.
- `python/dataset/` – dataset organization (environment and gunshot recordings) used for model training (see Data Availability).
- `figures_data/`, `metrics/` – plots, model metrics and training logs.
- `PCB/`, `lib/` – hardware design files and helper libraries.

## Data Availability (journal-ready statement)
All code required to build and run the device is included in this repository. The following datasets and artifacts are available as follows:

- Trained model: A quantized TensorFlow Lite model is included in `python/gunshot_model_quant.tflite` and additional exported models are in `python/exported_models/`.
- Processed training artifacts (training logs, metrics, and figure data) are included in `figures_data/` and `metrics/`.
- Raw audio recordings used for training (environmental and gunshot audio) are not included in this repository because of size and privacy/ethical constraints. Access to raw audio may be provided on reasonable request to the corresponding author for research purposes, subject to data sharing agreements.

If you need the raw audio files, preprocessed feature dumps, or a permanent DOI for the dataset, please contact the project maintainers (see Contact below). We can provide guidance to reproduce the training from raw data using the Jupyter notebook `python/model_training_quantization_jupyter.ipynb`.

Assumptions: The repository contains the trained model and training outputs, but large raw audio datasets are hosted externally or provided upon request. If you will publish this work in a journal and require a public dataset DOI, please tell us and we can prepare a dataset package for deposition (not currently included in this repo).

## Hardware
Components used in development
- Raspberry Pi Pico
- INMP441 digital microphone (I2S)
- NEO-6M GPS module
- NRF24L01 radio module
- IP5306 power-management IC

Power handling notes
- IP5306 `KEY` pin is driven by a PIO-based PWM to keep the power module enabled. The firmware contains a PIO state machine that pulses the `KEY` line with sufficient frequency and duty cycle to maintain power without manual intervention.
## Build and usage (Pico firmware)

Prerequisites

1. CMake >= 3.12
2. Raspberry Pi Pico SDK installed and `PICO_SDK_PATH` set in your environment

Quick build

1. From the firmware directory (for example `Raspberry_pi_pico/` or `inference-app/`):
   - mkdir build
   - cd build
   - cmake ..
   - make (or your platform's build tool)

2. Flashing

- Put the Pico into BOOTSEL mode and copy the generated `.uf2` file onto the Pico mass-storage device.

For exact build steps see the subdirectory `Raspberry_pi_pico/README.md` or `inference-app/README.md` (if present).

## Reproducing training
- A Jupyter notebook `python/model_training_quantization_jupyter.ipynb` demonstrates model training and quantization steps used to generate the included TFLite model. Use the `python/` directory for scripts and artifacts.

## Data sources and citations
The training used a combination of publicly available environmental and gunshot audio datasets. Below are the sources and citation information (BibTeX and IEEE formats).

1) E. Aydemir, "Gunshot audio dataset," Kaggle, 2021. [Online]. Available: https://www.kaggle.com/datasets/emrahaydemr/gunshot-audio-dataset. [Accessed: Oct. 11, 2025].

2) R. Kabealo and S. J. Wyatt, "Gunshot/Gunfire Audio Dataset," Zenodo, Jul. 2022, doi: 10.5281/zenodo.7004819. [Online]. Available: https://doi.org/10.5281/zenodo.7004819. [Accessed: Oct. 11, 2025].

3) K. J. Piczak, "ESC: Dataset for Environmental Sound Classification," in Proc. 23rd Annual ACM Conference on Multimedia, Brisbane, Australia, Oct. 2015, pp. 1015–1018, doi: 10.1145/2733373.2806390. [Online]. Available: http://dl.acm.org/citation.cfm?doid=2733373.2806390. [Accessed: Oct. 11, 2025].

## License and contact
- License: See `LICENSE` at the repository root for license terms.
- Contact / corresponding author: add a preferred contact email or GitHub handle here.


