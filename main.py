from app.cam_sender import start_picam_stream

if __name__ == "__main__":
	start_picam_stream(ip="10.10.10.93", port=8485)