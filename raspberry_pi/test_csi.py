import cv2

# 打开 CSI 摄像头，通常是 /dev/video0 或 /dev/video1
cap = cv2.VideoCapture(0)  # 如果是 /dev/video1，可以改成 1

if not cap.isOpened():
    print("Error: Unable to open CSI camera.")
else:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image.")
            break

        # 显示捕获的图像
        print("CSI Camera Frame Shape:", frame.shape)
        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
