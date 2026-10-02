# Real-Time Face Detection & Recognition on FPGA

Real-time face detection and recognition system running on a Xilinx Artix-7 XC7A35T FPGA. Captures a live OV7670 camera feed and displays results on a 640×480 VGA monitor with a bounding box and recognized identity overlay.

![Real-Time Face Detection & Recognition](assets/Result.png)

## How It Works

1. **Capture**: OV7670 feed converted RGB565 → RGB444 and clock-domain-crossed into a FIFO.
2. **Storage**: Ping-pong SRAM buffering feeds both the live display path and a dedicated grayscale frame buffer used for detection, so a full detection pass always sees one consistent frame.
3. **Detection (HOG + SVM)**: 64×64 sliding window scan (759 windows/frame) computes multiplier-free HOG features (zero-multiplier gradients, mean-threshold binarization) and scores them with a binary adder-tree linear SVM, followed by IoU-based non-max suppression.
4. **Recognition (LBPH)**: Detected face crops are compared against two enrolled identities using Local Binary Pattern histograms and L1 distance, with a rejection threshold for unknown faces.
5. **Display**: Detection/recognition results cross back into the video clock domain and are composited as an overlay on the live VGA output.

## Clock Domains

| Domain | Frequency | Purpose |
|---|---|---|
| `clk_pix` | 25.2 MHz | VGA timing, overlay |
| `clk_sys` | 50 MHz | SRAM control, detection/recognition |
| `clk_cam` | 24 MHz | Drives OV7670 XCLK |
| `cam_pclk` | ~24 MHz | Camera-returned pixel clock |

## Results

| Metric | Value |
|---|---|
| Detection pass latency | ~77.66 ms |
| Effective detection refresh rate | ~12.9 fps |
| LUT / FF / DSP utilization | 31.25% / 5.22% / 3.33% |
