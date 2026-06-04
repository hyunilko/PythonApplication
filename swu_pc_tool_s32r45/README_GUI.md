# SWU PC Tool (CustomTP) - PyQt6 GUI

## Features
- Load firmware image file
- Editable TransferBlockSize
- START button triggers SWU sequence
- Progress bar 0..100% updates after each ACK_DATA success
- DATA uses CustomTP segmentation (64-byte CAN FD frames: [HDR:2] + [PAYLOAD:62])
- CustomTP App-PDU format (Target 기대 포맷):
  - [payload_len(4B, little-endian)] + [msg_id(1B)] + [payload] + [padding(0..)]
  - msg_id for SWU DATA = 0x33
- Control/ACK uses single CAN frame

## Run
1) Install dependency
   pip install PyQt6

2) Run
   python swu_gui_qt6.py

## Notes
- SWU_REQUEST retry: every 100ms up to 50 times
- SWU_START / DOWNLOAD_START / VERIFY_IMAGE retry: every 100ms up to 3 times
- DATA block retry: up to 3 times on retryable failures
