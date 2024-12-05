import qrcode

# Replace this with your ngrok public URL
ngrok_url = "https://37f1-2403-d400-1001-11-28a0-e3a4-c92d-db5b.ngrok-free.app"

# Generate the QR code
qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_L,
    box_size=10,
    border=4,
)
qr.add_data(ngrok_url)
qr.make(fit=True)

# Save the QR code to a file
qr_code_image = qr.make_image(fill_color="black", back_color="white")
qr_code_image.save("qrcode.png")

print(f"QR code for {ngrok_url} has been saved as 'qrcode.png'.")
