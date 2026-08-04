# Local OCR models

These files are the pretrained models shipped with RapidOCR 3.9.2 and are used
through ONNX Runtime on CPU. RapidOCR is licensed under Apache-2.0.

| File | Purpose | SHA-256 |
| --- | --- | --- |
| `PP-OCRv6_det_small.onnx` | text detection | `090f04abcd9d9a7498bc4ebf677e4cb9bdce1fe4197ddb7e529f1ef44e1ff94f` |
| `ch_ppocr_mobile_v2.0_cls_mobile.onnx` | text orientation | `e47acedf663230f8863ff1ab0e64dd2d82b838fceb5957146dab185a89d6215c` |
| `PP-OCRv6_rec_small.onnx` | text recognition | `6f327246b50388f3c176ae304bd95767ea6dc0c9ae92153ef8cbe210b3c14884` |

The runtime passes explicit paths for all three files. No model download is
allowed or required during normal inference.
