# Feedback control of robots using AR Glasses and Gestures
This repository provides an interface for the Frame SDK (by Brilliant Labs) to detect hand gestures from a camera feed and apply gesture recongition with Google Mediapipe.


```python
@staticmethod
def from_image_bytes(img: Image, max_pixels = 48000, compress=False) -> 'TxSprite':
    """
    Create a sprite from the bytes of any image file format supported by PIL Image.open(), quantizing and scaling to ensure it fits within max_pixels (e.g. 48,000 pixels).
    """

    # Convert to RGB mode if not already
    if img.mode != 'RGB':
        img = img.convert('RGB')

    # Calculate new size to fit within max_pixels while maintaining aspect ratio
    img_pixels = img.width * img.height
    if img_pixels > max_pixels:
        scale_factor = (max_pixels / img_pixels) ** 0.5
        new_width = int(img.width * scale_factor)
        new_height = int(img.height * scale_factor)
        img = img.resize((new_width, new_height), Image.Resampling.LANCZOS)

    # Ensure the image does not exceed 640x400 after initial scaling
    if img.width > 640 or img.height > 400:
        img.thumbnail((640, 400), Image.Resampling.NEAREST)

    # Quantize to 16 colors if needed
    if img.mode != 'P' or img.getcolors() is None or len(img.getcolors()) > 16:
        img = img.quantize(colors=16, method=Image.Quantize.MEDIANCUT)

    # Get first 16 RGB colors from the palette
    palette = list(img.getpalette()[:48])
    pixel_data = np.array(img)

    # The quantized palette comes back in a luminance gradient from lightest to darkest.
    # Ensure the darkest is at index 0 by swapping index 0 with index 15
    palette[0:3], palette[45:48] = palette[45:48], palette[0:3]

    # Update the pixel_data accordingly
    pixel_data[pixel_data == 0] = 255  # Temporary value to avoid conflict
    pixel_data[pixel_data == 15] = 0
    pixel_data[pixel_data == 255] = 15

    # Set the first (darkest, not necessarily black) entry in the palette to black for transparency
    palette[0:3] = 0, 0, 0

    return TxSprite(
        width=img.width,
        height=img.height,
        num_colors=16,
        palette_data=bytes(palette),
        pixel_data=pixel_data.tobytes(),
        compress=compress
    )

```

The above is a modification to tx_sprite.py in frame_msg library.