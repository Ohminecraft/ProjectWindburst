// English texts of the configurator. Key = the Vietnamese string exactly as it is written in the code (js/*.js and js/labels.js).
// {name} parts are filled in by tr() and must be kept. app/tests/test_gui_i18n.py checks that every string of the page is
// either translated here or listed in SAME, that no entry is left over, and that the {name} parts match.
export const EN = {
  "Không gọi được chương trình wbclient. Cửa sổ lệnh chạy \"python -m wbclient gui\" còn mở không?":
    "Cannot reach the wbclient program. Is the terminal running \"python -m wbclient gui\" still open?",
  "Hình 3D của máy theo tư thế đang đo":
    "3D view of the craft following the measured attitude",
  "Đồ thị ":
    "Chart of ",
  "bây giờ":
    "now",
  "chưa có dữ liệu":
    "no data yet",
  "Cổng nối với flight controller":
    "Port of the flight controller",
  "Tốc độ baud":
    "Baud rate",
  "Tốc độ baud của cổng UART. Mạch dùng USB gốc của ESP32-S3 thì chọn số nào cũng được.":
    "Baud rate of the UART port. On a board that uses the native USB of the ESP32-S3 any value works.",
  "Kết nối":
    "Connect",
  "Tìm lại các cổng":
    "Look for ports again",
  "Tìm cổng":
    "Find ports",
  "Chưa kết nối":
    "Not connected",
  "Lưu":
    "Save",
  "Bỏ thay đổi":
    "Discard changes",
  "Mô phỏng wb_sim (127.0.0.1:5760)":
    "Simulator wb_sim (127.0.0.1:5760)",
  "Tên cổng (ví dụ COM7, /dev/ttyACM0, hoặc socket://127.0.0.1:5760):":
    "Port name (for example COM7, /dev/ttyACM0, or socket://127.0.0.1:5760):",
  "Không thấy cổng nào":
    "No port found",
  "Nhập cổng khác…":
    "Type another port…",
  "Chưa có cổng nào để kết nối. Cắm cáp USB rồi bấm \"Tìm cổng\".":
    "There is no port to connect to. Plug in the USB cable, then press \"Find ports\".",
  "Đã lưu. Đang khởi động lại flight controller…":
    "Saved. Restarting the flight controller…",
  "Đã lưu và khởi động lại.":
    "Saved and restarted.",
  "Đã lưu vào flash.":
    "Saved to flash.",
  "Bỏ mọi thay đổi chưa lưu? Flight controller sẽ khởi động lại và nạp lại cấu hình trong flash.":
    "Discard every unsaved change? The flight controller restarts and loads the settings stored in flash.",
  "Đã nạp lại cấu hình trong flash.":
    "Settings reloaded from flash.",
  "Mất kết nối với cổng {port} (rút cáp?)":
    "Lost the port {port} (cable unplugged?)",
  "Firmware không trả lời (đang khởi động lại, treo, hoặc mất nguồn?)":
    "The firmware does not answer (restarting, hung, or without power?)",
  "Flight controller vừa khởi động lại":
    "The flight controller has just restarted",
  "Đang xử lý…":
    "Working…",
  "Ngắt kết nối":
    "Disconnect",
  "Mất liên lạc với chương trình wbclient trên máy này. Chạy lại \"python -m wbclient gui\" rồi tải lại trang.":
    "Lost contact with the wbclient program on this computer. Run \"python -m wbclient gui\" again, then reload this page.",
  "Firmware không trả lời.":
    "The firmware does not answer.",
  "Đang ARM. Firmware khóa mọi thay đổi cấu hình cho tới khi disarm.":
    "ARMED. The firmware refuses every settings change until it is disarmed.",
  "Đang chờ dữ liệu":
    "Waiting for data",
  "Chưa arm được":
    "Cannot arm",
  "Sẵn sàng arm":
    "Ready to arm",
  "{n} khung/s":
    "{n} frames/s",
  "có sóng":
    "link up",
  "mất sóng":
    "no link",
  "Pin":
    "Battery",
  "không đo":
    "not measured",
  "Vòng lặp":
    "Loop",
  "đang đo":
    "measuring",
  "Lưu và khởi động lại":
    "Save and restart",
  "Đang đọc cấu hình từ flight controller…":
    "Reading the settings from the flight controller…",
  "Lỗi khi dựng tab: ":
    "Error while building the tab: ",
  "Cổng đang bị chương trình khác giữ? Đóng serial monitor của PlatformIO / Arduino IDE, hoặc cửa sổ configurator khác.":
    "Is another program holding the port? Close the serial monitor of PlatformIO / Arduino IDE, or another configurator window.",
  "Rút cáp, cắm lại, rồi bấm \"Tìm cổng\": tên cổng có thể đã đổi.":
    "Unplug the cable, plug it in again, then press \"Find ports\": the port name may have changed.",
  "Chọn đúng cổng của flight controller chưa? Cổng có chip Espressif / CP210x / CH34x được xếp lên đầu danh sách.":
    "Is this the port of the flight controller? Ports with an Espressif / CP210x / CH34x chip are listed first.",
  "Mạch dùng cầu USB-UART thì baud phải khớp với firmware (mặc định 230400). Thử baud khác ở ô bên cạnh ô cổng.":
    "With a USB-UART bridge the baud rate must match the firmware (230400 by default). Try another baud rate in the box next to the port.",
  "Mạch đã nạp firmware Windburst chưa? Nếu vừa nạp xong, bấm nút reset rồi kết nối lại.":
    "Has the Windburst firmware been flashed? If you just flashed it, press the reset button and connect again.",
  "Nạp firmware build từ đúng thư mục dự án này (cùng protocol/wbp_schema.json), hoặc dùng app đi kèm bản firmware đó.":
    "Flash the firmware built from this project folder (same protocol/wbp_schema.json), or use the app that came with that firmware.",
  "Lần kết nối vừa rồi không được":
    "The last connection attempt failed",
  "Cổng đã gửi về những dữ liệu sau (không phải khung WBP). Chữ đọc được là log khởi động hoặc lỗi của ESP32; toàn ký tự lạ thường là sai baud.":
    "The port sent the data below (not WBP frames). Readable text is the boot log or an error of the ESP32; only odd characters usually means the wrong baud rate.",
  "Cổng không gửi về byte nào.":
    "The port sent no bytes at all.",
  "Nối flight controller để bắt đầu":
    "Connect the flight controller to begin",
  "Cắm cáp USB vào mạch ESP32-S3 đã nạp firmware Windburst.":
    "Plug the USB cable into the ESP32-S3 board that runs the Windburst firmware.",
  "Bấm \"Tìm cổng\", chọn cổng COM của mạch ở thanh trên cùng, rồi bấm \"Kết nối\".":
    "Press \"Find ports\", choose the COM port of the board in the top bar, then press \"Connect\".",
  "Tab Tổng quan cho biết firmware tìm thấy cảm biến nào và vì sao chưa arm được.":
    "The Overview tab shows which sensors the firmware found and why it cannot arm yet.",
  "Chưa có mạch? Chạy build/wb_sim (firmware thật trên mạch ảo), bấm \"Tìm cổng\" và chọn \"Mô phỏng wb_sim\".":
    "No board yet? Run build/wb_sim (the real firmware on a virtual board), press \"Find ports\" and choose \"Simulator wb_sim\".",
  "Đường truyền là nhị phân nên không dùng chung được với serial monitor: đóng serial monitor trước khi kết nối.":
    "The link is binary, so it cannot be shared with a serial monitor: close the serial monitor before connecting.",
  "Đầu dưới của dải":
    "Lower end of the range",
  "Đầu trên của dải":
    "Upper end of the range",
  "Mode":
    "Modes",
  "Gán công tắc AUX cho từng mode. Vạch đậm trên thanh là vị trí công tắc lúc này: gạt công tắc để xem nó rơi vào đâu.":
    "Assign an AUX switch to each mode. The dark mark on the bar is where the switch is right now: flip it to see where it lands.",
  "Tắt":
    "Off",
  "Kênh AUX":
    "AUX channel",
  "Xóa dải này":
    "Remove this range",
  "Xóa":
    "Remove",
  "Thêm dải":
    "Add range",
  "Đã dùng hết 16 dải. Xóa bớt một dải trước.":
    "All 16 ranges are in use. Remove one first.",
  "Chưa gán công tắc ARM: máy sẽ không arm được.":
    "No ARM switch assigned: the craft cannot be armed.",
  "Đang bật":
    "On",
  "Khung {layout} nhìn từ trên xuống, {n} motor":
    "Frame {layout} seen from above, {n} motors",
  "mũi":
    "nose",
  "Mũi tên là chiều mỗi cánh PHẢI quay (nhìn từ trên xuống). Vòng tô đậm là lệnh đang gửi tới ESC.":
    "The arrow is the direction each propeller MUST turn (seen from above). The filled ring is the command being sent to the ESC.",
  "Tất cả motor":
    "All motors",
  "Dừng tất cả":
    "Stop all",
  "Đã tháo hết cánh quạt.":
    "All propellers are removed.",
  " Motor sẽ quay thật khi kéo thanh trượt.":
    " The motors really spin when you move a slider.",
  "Tất cả":
    "All",
  "Kéo từng thanh để biết motor số mấy nằm ở đâu và quay chiều nào. Sai vị trí thì sửa \"Thứ tự motor\"; sai chiều thì bật \"Đảo chiều\" cho motor đó.":
    "Move each slider to see where that motor number sits and which way it turns. Wrong position: fix the hardware slot; wrong direction: switch \"Reversed\" on for that motor.",
  "Slot phần cứng của motor {n}":
    "Hardware slot of motor {n}",
  "Đảo chiều motor {n}":
    "Reverse motor {n}",
  "Đã gửi lệnh đổi chiều cho ESC {n}. Bấm \"Lưu\" để giữ.":
    "Direction command sent to ESC {n}. Press \"Save\" to keep it.",
  "Chỉ đổi chiều bằng lệnh được với DShot. Với PWM / Oneshot hãy đổi hai dây motor.":
    "The direction can only be changed by command with DShot. With PWM / Oneshot swap two motor wires.",
  "Kêu":
    "Beep",
  "ESC chỉ kêu được qua DShot.":
    "The ESC can only beep over DShot.",
  "Từng motor":
    "Per motor",
  "Lệnh":
    "Command",
  "Lỗi telemetry":
    "Telemetry errors",
  "Nhiệt độ ESC":
    "ESC temperature",
  "Slot phần cứng":
    "Hardware slot",
  "Đảo chiều":
    "Reversed",
  "Slot phần cứng: motor này nối vào chân \"Slot\" nào ở tab Chân và cổng. RPM có khi bật DShot hai chiều hoặc dây telemetry ESC; lỗi telemetry chỉ với DShot hai chiều. Dòng: extended DShot telemetry hoặc dây telemetry ESC.":
    "Hardware slot: which \"Slot\" pin of the Pins and ports tab this motor is wired to. RPM comes with bidirectional DShot or the ESC telemetry wire; telemetry errors only with bidirectional DShot. Current: extended DShot telemetry or the ESC telemetry wire.",
  "Điện áp ESC báo: ":
    "Voltage reported by the ESC: ",
  "không có":
    "none",
  "Gyro thô":
    "Raw gyro",
  "Sau bộ lọc":
    "Filtered",
  "Độ rung (gyro)":
    "Vibration (gyro)",
  " Bật đồ thị (500 mẫu/giây)":
    " Show the chart (500 samples/s)",
  "Motor":
    "Motors",
  "Kiểu khung, ESC, chiều quay và thứ tự motor. Tháo cánh trước khi test.":
    "Frame type, ESC, spin direction and motor order. Remove the propellers before testing.",
  "Khung":
    "Frame",
  "Kiểu khung, giao thức ESC và thứ tự motor có hiệu lực sau khi lưu và khởi động lại. DShot qua RMT của ESP32-S3 chạy được tối đa 4 motor.":
    "Frame type, ESC protocol and motor order take effect after save and restart. DShot over the RMT of the ESP32-S3 drives at most 4 motors.",
  "La bàn":
    "Compass",
  "firmware không báo cáo":
    "not reported by the firmware",
  "Không có":
    "Missing",
  "Có":
    "Found",
  "Trạng thái":
    "State",
  "Firmware báo":
    "Firmware says",
  "Tổng quan":
    "Overview",
  "Firmware tìm thấy gì lúc khởi động, vì sao chưa arm được, và các số đo đang chạy.":
    "What the firmware found at start-up, why it cannot arm yet, and the live readings.",
  "Quay mũi máy về phía xa":
    "Point the nose away from me",
  "Đang ARM":
    "ARMED",
  "Không còn gì chặn. Gạt công tắc ARM để bật motor.":
    "Nothing blocks arming. Flip the ARM switch to start the motors.",
  "Hỏi lại firmware":
    "Ask the firmware again",
  "Tư thế":
    "Attitude",
  "Nghiêng máy sang phải thì roll dương, chúi mũi xuống thì pitch dương, quay mũi sang phải thì yaw tăng. Hình phải nghiêng đúng như máy trên tay; nếu sai, sửa \"Hướng chip IMU\" ở tab Cảm biến.":
    "Tilt the craft to the right and roll is positive, push the nose down and pitch is positive, turn the nose to the right and yaw grows. The picture must lean exactly like the craft in your hand; if not, fix \"IMU chip orientation\" on the Sensors tab.",
  "Vì sao chưa arm được":
    "Why it cannot arm",
  "Số đo":
    "Readings",
  "La bàn (gauss)":
    "Compass (gauss)",
  "không có số đo":
    "no reading",
  "Độ cao ước lượng":
    "Estimated height",
  "chưa hợp lệ":
    "not valid yet",
  "fix {fix}D, {sats} vệ tinh, HDOP {hdop}":
    "{fix}D fix, {sats} satellites, HDOP {hdop}",
  "chưa fix, {sats} vệ tinh ({state})":
    "no fix, {sats} satellites ({state})",
  "Thời gian một chu kỳ":
    "Time of one cycle",
  "{us} µs, dài nhất {max} µs":
    "{us} µs, longest {max} µs",
  "Chu kỳ điều khiển thực tế":
    "Control cycles really run",
  "IMU qua SPI":
    "IMU on SPI",
  "Điền chân CS thì firmware dùng SPI; để trống cả bốn thì IMU chạy I2C trên bus cảm biến bên dưới.":
    "With a CS pin the firmware uses SPI; leave all four empty and the IMU runs on I2C on the sensor bus below.",
  "INT (chưa dùng)":
    "INT (not used yet)",
  "Bus I2C cảm biến":
    "Sensor I2C bus",
  "Barometer, lidar, la bàn (và IMU nếu không có chân CS).":
    "Barometer, lidar, compass (and the IMU when it has no CS pin).",
  "RX của ESP32 nối với TX của module GPS.":
    "RX of the ESP32 goes to TX of the GPS module.",
  "Receiver nối tiếp (UART2)":
    "Serial receiver (UART2)",
  "CRSF / SBUS / IBUS / Spektrum. TX chỉ cần khi dùng telemetry.":
    "CRSF / SBUS / IBUS / Spektrum. TX is only needed for telemetry.",
  "Khác":
    "Other",
  "ADC đo pin":
    "Battery ADC",
  "chân strapping (BOOT)":
    "a strapping pin (BOOT)",
  "chân strapping":
    "a strapping pin",
  "UART0 TX (cổng nối app nếu dùng cầu USB-UART)":
    "UART0 TX (the app link when a USB-UART bridge is used)",
  "UART0 RX (cổng nối app nếu dùng cầu USB-UART)":
    "UART0 RX (the app link when a USB-UART bridge is used)",
  "{label}: GPIO{pin} không tồn tại trên ESP32-S3 (0 đến 48).":
    "{label}: GPIO{pin} does not exist on the ESP32-S3 (0 to 48).",
  "{label}: GPIO{pin} thường là {what}.":
    "{label}: GPIO{pin} is usually {what}.",
  "GPIO{pin} đang gán cho cả \"{a}\" và \"{b}\".":
    "GPIO{pin} is assigned to both \"{a}\" and \"{b}\".",
  "IMU SPI: đã có CS nhưng còn thiếu MISO, MOSI hoặc SCK.":
    "IMU SPI: CS is set but MISO, MOSI or SCK is missing.",
  "Bus I2C: cần cả SDA và SCL.":
    "I2C bus: both SDA and SCL are needed.",
  "Chân và cổng":
    "Pins and ports",
  "Gán GPIO cho từng module. Ô để trống nghĩa là không dùng (hoặc dùng chân mặc định ghi sẵn trong hardware_config.h).":
    "Assign a GPIO to each module. An empty box means not used (or the default pin written in hardware_config.h).",
  "Không thấy chân trùng hay chân đặc biệt.":
    "No duplicate or special pins found.",
  "Chân được lưu trong flash của flight controller, không nằm trong mã nguồn. Sau khi sửa, bấm \"Lưu và khởi động lại\" ở góc trên: firmware chỉ đọc bảng chân lúc khởi động.":
    "Pins are stored in the flash of the flight controller, not in the source code. After a change press \"Save and restart\" at the top: the firmware reads the pin table only at start-up.",
  "Theo thứ tự slot phần cứng. Motor nào dùng slot nào chỉnh ở tab Motor (thứ tự motor).":
    "In hardware slot order. Which motor uses which slot is set on the Motors tab.",
  "Kiểm tra":
    "Check",
  "Cổng nối tiếp":
    "Serial ports",
  "Cổng":
    "Port",
  "Chức năng":
    "Function",
  "{name} (kênh {n})":
    "{name} (channel {n})",
  "Kiểm tra từng cần và công tắc trên tay điều khiển trước khi gán mode.":
    "Check every stick and switch of the transmitter before assigning modes.",
  "Chọn nhanh…":
    "Quick choice…",
  "(0 = kênh 1)":
    "(0 = channel 1)",
  "Kênh đang nhận":
    "Channels being received",
  "Tín hiệu: ":
    "Link: ",
  "có":
    "up",
  "mất":
    "lost",
  "Chất lượng: ":
    "Quality: ",
  "Tốc độ khung: ":
    "Frame rate: ",
  "Thang 900 đến 2100 µs, vạch giữa là 1500. Cần ga thấp nhất phải về gần 1000.":
    "Scale 900 to 2100 µs, the middle mark is 1500. The throttle stick at its lowest must read close to 1000.",
  "Giao thức":
    "Protocol",
  "CRSF dùng cho mọi receiver ELRS và Crossfire nối qua UART. Chân UART của receiver đặt ở tab Chân và cổng. Có chân TX (nối vào RX của receiver) thì firmware gửi telemetry CRSF về tay phát: pin, dòng, mAh, tư thế, chế độ bay, GPS, vario. Đổi giao thức cần lưu và khởi động lại.":
    "CRSF is for every ELRS and Crossfire receiver on a UART. The receiver's UART pins are set in the Pins and ports tab. With a TX pin (wired to the receiver's RX) the firmware sends CRSF telemetry back to the transmitter: battery, current, mAh, attitude, flight mode, GPS, vario. Changing the protocol needs a save and a restart.",
  "Thứ tự kênh":
    "Channel order",
  "Kiểu thứ tự":
    "Order preset",
  "Tính và áp dụng":
    "Compute and apply",
  "Đã áp dụng hiệu chuẩn accelerometer. Bấm \"Lưu\" để giữ lại.":
    "Accelerometer calibration applied. Press \"Save\" to keep it.",
  "Chưa đủ 6 mặt.":
    "Not all 6 faces yet.",
  "Số đo 6 mặt không hợp lý (chip lỗi hoặc máy bị động). Hủy và làm lại.":
    "The 6 readings do not make sense (faulty chip, or the craft moved). Cancel and start again.",
  "Bắt đầu":
    "Start",
  "Đang thu mẫu: xoay máy chậm quanh mọi trục, như vẽ số 8.":
    "Collecting samples: turn the craft slowly around every axis, like drawing a figure 8.",
  "Hiệu chuẩn la bàn thành công. Bấm \"Lưu\" để giữ lại.":
    "Compass calibration done. Press \"Save\" to keep it.",
  "Đã áp dụng hiệu chuẩn la bàn.":
    "Compass calibration applied.",
  "thất bại":
    "failed",
  "Chưa được: {why}. Xoay tiếp rồi bấm lại.":
    "Not yet: {why}. Keep turning, then press again.",
  "Chưa bấm \"Bắt đầu\".":
    "\"Start\" was not pressed yet.",
  "Hủy":
    "Cancel",
  "đang chạy {n} %":
    "running, {n} %",
  "đã hiệu chuẩn":
    "calibrated",
  "chưa xong (máy phải đứng yên)":
    "not done (the craft must be still)",
  "Đã tính xong và đang dùng hiệu chuẩn mới.":
    "Computed; the new calibration is in use.",
  "Đang lấy mẫu {n} %, giữ yên…":
    "Sampling, {n} %, hold still…",
  "Mặt này chưa lấy được: máy bị động hoặc chưa nằm hẳn lên một mặt. Đặt lại rồi bấm lần nữa.":
    "This face was not captured: the craft moved or does not rest fully on one face. Put it down again and press once more.",
  "Đủ 6 mặt. Bấm \"Tính và áp dụng\".":
    "All 6 faces captured. Press \"Compute and apply\".",
  "Đã có {n}/6 mặt. Xoay máy sang mặt khác rồi bấm \"Lấy mặt này\".":
    "{n}/6 faces captured. Turn the craft onto another face, then press \"Capture this face\".",
  "Đang dùng hiệu chuẩn: offset {offset} g, scale {scale}.":
    "Calibration in use: offset {offset} g, scale {scale}.",
  "Chưa có hiệu chuẩn accelerometer (đang dùng số đo thô của chip).":
    "No accelerometer calibration (the raw chip readings are used).",
  "Đang thu mẫu ({n} % độ phủ): xoay máy chậm quanh mọi trục, như vẽ số 8.":
    "Collecting samples ({n} % coverage): turn the craft slowly around every axis, like drawing a figure 8.",
  "Đang dùng hiệu chuẩn la bàn (bán kính {r} gauss).":
    "Compass calibration in use (radius {r} gauss).",
  "La bàn chưa hiệu chuẩn.":
    "The compass is not calibrated.",
  "Hiệu chuẩn gyro":
    "Calibrate gyro",
  "Đang hiệu chuẩn gyro, giữ máy đứng yên khoảng 1 giây.":
    "Calibrating the gyro: keep the craft still for about 1 second.",
  "Firmware tự làm việc này mỗi lần khởi động. Chỉ cần bấm lại nếu máy bị động lúc cấp nguồn.":
    "The firmware does this at every start-up. Press again only if the craft was moved while powering up.",
  "Accelerometer, 6 mặt":
    "Accelerometer, 6 faces",
  "Đặt máy lần lượt lên 6 mặt: nằm bình thường, lật úp, dựng mũi lên trời, chúi mũi xuống đất, nghiêng hẳn sang trái, nghiêng hẳn sang phải. Thứ tự tùy ý, firmware tự nhận ra mặt nào. Mỗi mặt giữ yên rồi bấm \"Lấy mặt này\".":
    "Rest the craft on its 6 faces in turn: upright, upside down, nose to the sky, nose to the ground, fully on its left side, fully on its right side. Any order: the firmware recognises the face. Hold each face still and press \"Capture this face\".",
  "Lấy mặt này":
    "Capture this face",
  "Đang lấy mẫu mặt trước, chờ một giây.":
    "Still sampling the previous face: wait a second.",
  "Cân bằng (level trim)":
    "Level trim",
  "Đặt máy trên mặt phẳng ngang, giữ yên, rồi bấm. Firmware ghi độ lệch còn lại để Angle mode giữ máy thật phẳng. Lệch quá 20° thì đó là hướng lắp chip, sửa ở \"Hướng chip IMU\".":
    "Put the craft on a level surface, keep it still, then press. The firmware stores the remaining offset so that Angle mode holds the craft truly level. More than 20° off is a mounting orientation: fix it with \"IMU chip orientation\".",
  "Lấy tư thế này làm phẳng":
    "Take this attitude as level",
  "Đã lấy level trim. Bấm \"Lưu\" để giữ lại.":
    "Level trim taken. Press \"Save\" to keep it.",
  "Không lấy được: máy đang động, hoặc nghiêng quá 20°.":
    "Not taken: the craft is moving, or it is tilted more than 20°.",
  "Làm ngoài trời hoặc xa kim loại và dây nguồn. Bấm \"Bắt đầu\", xoay máy chậm theo mọi hướng cho các chấm phủ kín hai vòng tròn, rồi bấm \"Tính và áp dụng\".":
    "Do this outdoors, or away from metal and power wires. Press \"Start\", turn the craft slowly in every direction until the dots fill both circles, then press \"Compute and apply\".",
  "Cảm biến":
    "Sensors",
  "Hiệu chuẩn, hướng lắp chip và bật tắt từng module.":
    "Calibration, chip orientation and the switch of each module.",
  "IMU và vòng điều khiển":
    "IMU and control loop",
  "Các mục này có hiệu lực sau khi lưu và khởi động lại.":
    "These take effect after save and restart.",
  "Bật hoặc tắt module cần lưu và khởi động lại. Module bật mà firmware không tìm thấy sẽ hiện \"Không có\" ở tab Tổng quan kèm lý do.":
    "Switching a module on or off needs save and restart. A module that is on but not found shows \"Missing\" on the Overview tab, with the reason.",
  "{fix}D, {sats} vệ tinh, HDOP {hdop}":
    "{fix}D, {sats} satellites, HDOP {hdop}",
  "chưa fix, {sats} vệ tinh":
    "no fix, {sats} satellites",
  "Vị trí":
    "Position",
  "{lat}, {lon}, cao {alt} m":
    "{lat}, {lon}, altitude {alt} m",
  "cách {d} m, hướng {dir}°":
    "{d} m away, bearing {dir}°",
  "cách 0 m":
    "0 m away",
  "Đặt home tại đây":
    "Set home here",
  "Đã đặt home.":
    "Home set.",
  "GPS chưa có fix đủ tốt để đặt home.":
    "The GPS fix is not good enough to set home.",
  "Hệ thống":
    "System",
  "Tải bản sao lưu (.json)":
    "Download a backup (.json)",
  "Nạp bản sao lưu…":
    "Load a backup…",
  "Đã nạp {n} mục vào RAM. Kiểm tra lại rồi bấm \"Lưu\" ở góc trên để ghi vào flash.":
    "{n} sections loaded into RAM. Check them, then press \"Save\" at the top to write them to flash.",
  "Firmware đã giới hạn: ":
    "Limited by the firmware: ",
  "{field} ({asked} thành {stored})":
    "{field} ({asked} became {stored})",
  "Bỏ qua vì firmware này không có: ":
    "Skipped because this firmware does not have them: ",
  "File không phải JSON hợp lệ.":
    "The file is not valid JSON.",
  "Tất cả các mục":
    "All sections",
  "Về mặc định":
    "Reset to defaults",
  "TOÀN BỘ cấu hình (kể cả chân và hiệu chuẩn)":
    "ALL settings (pins and calibration included)",
  "Đưa {what} về mặc định? Thay đổi nằm trong RAM cho tới khi bạn bấm \"Lưu\".":
    "Reset {what} to defaults? The change stays in RAM until you press \"Save\".",
  "Đã về mặc định (chưa lưu).":
    "Reset to defaults (not saved yet).",
  "Chưa có dòng nào.":
    "No lines yet.",
  "Chưa có byte nào ngoài giao thức.":
    "No bytes outside the protocol so far.",
  "Tên":
    "Name",
  "Phiên bản":
    "Version",
  "Mã schema":
    "Schema hash",
  "Tính năng":
    "Features",
  "Sao lưu và khôi phục":
    "Backup and restore",
  "Bản sao lưu chứa mọi mục cấu hình, kể cả bảng chân và kết quả hiệu chuẩn.":
    "A backup holds every settings section, the pin table and the calibration results included.",
  "Nhật ký firmware":
    "Firmware log",
  "Dữ liệu thô từ cổng serial":
    "Raw data from the serial port",
  "Những byte không thuộc khung WBP: log khởi động của ESP32, thông báo lỗi khi firmware treo (\"Guru Meditation\", backtrace), hoặc dữ liệu của chương trình khác. Bình thường ô này trống.":
    "Bytes that are not WBP frames: the boot log of the ESP32, the error printed when the firmware crashes (\"Guru Meditation\", backtrace), or output of another program. Normally this box is empty.",
  "Cần ở {n} %: ":
    "Stick at {n} %: ",
  "Cần lúc này: ":
    "Sticks now: ",
  "Đường cong rate của ba trục":
    "Rate curves of the three axes",
  "Trục ngang: độ lệch cần (sau deadband). Trục dọc: tốc độ quay yêu cầu, °/s. Chấm tròn là vị trí cần lúc này.":
    "Horizontal: stick deflection (after deadband). Vertical: commanded rotation rate, °/s. The dot is where the stick is now.",
  "Cần ga {stick} % cho lệnh ga {cmd} %":
    "Throttle stick {stick} % gives throttle command {cmd} %",
  "Đường cong ga":
    "Throttle curve",
  "Lệnh ga theo cần ga":
    "Throttle command over throttle stick",
  "PID từng trục":
    "PID per axis",
  "Bộ lọc":
    "Filters",
  "Rate (kiểu Actual)":
    "Rates (Actual type)",
  "Độ nhạy giữa (°/s)":
    "Center sensitivity (°/s)",
  "Tốc độ tối đa (°/s)":
    "Max rate (°/s)",
  "Ga":
    "Throttle",
  "Đường cong ga chỉ dùng khi bay ga tay. Altitude Hold dùng thẳng vị trí cần ga, theo mục bên dưới.":
    "The throttle curve is only for manual throttle. Altitude Hold uses the stick position itself, as set below.",
  "Angle và Stabilize":
    "Angle and Stabilize",
  "Không mở được cổng {port}: {detail}":
    "Cannot open the port {port}: {detail}",
  "Mở được cổng {port} ở {baud} baud nhưng firmware Windburst không trả lời.":
    "The port {port} opened at {baud} baud, but no Windburst firmware answered.",
  "Firmware và app không cùng phiên bản giao thức: {detail}":
    "The firmware and the app do not speak the same protocol version: {detail}",
  "Giá trị không hợp lệ: {detail}":
    "Invalid value: {detail}",
  "File không phải bản sao lưu settings của Windburst.":
    "The file is not a Windburst settings backup.",
  "(không dùng)":
    "(not used)",
  " (firmware chưa hỗ trợ)":
    " (not supported by the firmware yet)",
  "trống":
    "empty",
  "Có hiệu lực sau khi lưu và khởi động lại.":
    "Takes effect after save and restart.",
  "Tần số bộ ước lượng":
    "Estimator rate",
  "Góc nghiêng tối đa cho phép arm":
    "Largest tilt that may arm",
  "0 = tắt.":
    "0 = off.",
  "Hướng chip IMU trên mạch":
    "IMU chip orientation",
  "Chỉnh tinh roll":
    "Fine trim, roll",
  "Chỉnh tinh pitch":
    "Fine trim, pitch",
  "Chỉnh tinh yaw":
    "Fine trim, yaw",
  "Chờ trước khi vào failsafe":
    "Wait before failsafe",
  "Mất sóng lâu hơn mức này thì bắt đầu hạ cánh.":
    "A link loss longer than this starts the descent.",
  "Sóng phải ổn định lại trong":
    "The link must be stable again for",
  "Ga khi không có ước lượng độ cao":
    "Throttle without a height estimate",
  "% ga hover":
    "% of hover throttle",
  "Giao thức receiver":
    "Receiver protocol",
  "AUX1 bắt đầu từ kênh":
    "AUX1 starts at channel",
  "8 kênh AUX liên tiếp tính từ kênh này.":
    "8 consecutive AUX channels counted from this one.",
  "Deadband cần điều khiển":
    "Stick deadband",
  "Coi là mất sóng sau":
    "Link counts as lost after",
  "% ga":
    "% throttle",
  "Kiểu khung":
    "Frame type",
  "Cánh quay ra ngoài (props out)":
    "Props out",
  "Đảo chiều quay của mọi motor so với hình vẽ mặc định.":
    "Reverses the spin of every motor compared with the default picture.",
  "Giao thức ESC":
    "ESC protocol",
  "Xung khi chưa arm":
    "Pulse while disarmed",
  "Xung khi arm (ga thấp nhất)":
    "Pulse when armed (lowest throttle)",
  "Xung ga tối đa":
    "Pulse at full throttle",
  "Tần số xung":
    "Pulse frequency",
  "0 = mặc định của giao thức.":
    "0 = protocol default.",
  "DShot hai chiều (đọc RPM)":
    "Bidirectional DShot (reads RPM)",
  "Điện áp, dòng, nhiệt độ từ ESC. Cần DShot hai chiều. Firmware tự bật chức năng này trên ESC khi ESC bắt đầu trả lời (lúc cắm pin), không bao giờ bật khi đang arm.":
    "Voltage, current and temperature from the ESC. Needs bidirectional DShot. The firmware switches it on in the ESCs when they start answering (when the battery is plugged in), never while armed.",
  "Số cực motor":
    "Motor poles",
  "Đếm số nam châm trên chuông motor (thường 14).":
    "Count the magnets on the motor bell (usually 14).",
  "× 0,1 %":
    "× 0.1 %",
  "55 = 5,5 %.":
    "55 = 5.5 %.",
  "Giới hạn công suất motor":
    "Motor output limit",
  "Dynamic idle, RPM tối thiểu":
    "Dynamic idle, minimum RPM",
  "0 = tắt. Cần DShot hai chiều.":
    "0 = off. Needs bidirectional DShot.",
  "Dynamic idle, mức tăng tối đa":
    "Dynamic idle, max increase",
  "Lệnh đảo chiều kiểu AM32 (20/21)":
    "AM32-style direction commands (20/21)",
  "Tắt = lệnh DShot 7/8 (BLHeli_S, Bluejay, BLHeli_32).":
    "Off = DShot commands 7/8 (BLHeli_S, Bluejay, BLHeli_32).",
  "Loại la bàn":
    "Compass type",
  "Hướng chip la bàn":
    "Compass chip orientation",
  "La bàn, chỉnh tinh roll":
    "Compass fine trim, roll",
  "La bàn, chỉnh tinh pitch":
    "Compass fine trim, pitch",
  "La bàn, chỉnh tinh yaw":
    "Compass fine trim, yaw",
  "Độ lệch từ (đông là dương)":
    "Magnetic declination (east positive)",
  "Loại lidar":
    "Lidar type",
  "VL53L1X: tầm đo":
    "VL53L1X: range mode",
  "0 gần, 1 vừa, 2 xa.":
    "0 short, 1 medium, 2 long.",
  "Giao thức GPS":
    "GPS protocol",
  "Tần số cập nhật":
    "Update rate",
  "Tự dò baud":
    "Auto baud",
  "Tự cấu hình module":
    "Auto configure the module",
  "Dùng Galileo":
    "Use Galileo",
  "Mô hình động học khi dò vệ tinh":
    "Dynamic model while acquiring",
  "Mô hình động học khi bay":
    "Dynamic model in flight",
  "Chuẩn UTC":
    "UTC standard",
  "Chỉ đặt home một lần":
    "Set home only once",
  "Dùng tốc độ 3D":
    "Use 3D speed",
  "Số vệ tinh tối thiểu":
    "Minimum satellites",
  "Âm lượng":
    "Volume",
  "không có IMU":
    "no IMU",
  "gyro chưa hiệu chuẩn xong (giữ máy đứng yên)":
    "the gyro calibration is not finished (keep the craft still)",
  "không có tín hiệu receiver":
    "no receiver signal",
  "cần ga chưa về thấp nhất":
    "the throttle stick is not at its lowest",
  "máy đang nghiêng quá góc cho phép":
    "the craft is tilted more than allowed",
  "đang failsafe":
    "failsafe is active",
  "Altitude Hold đang bật nhưng không có barometer (và không có GPS fix để thay)":
    "Altitude Hold is on but there is no barometer (and no GPS fix to stand in)",
  "công tắc ARM bật sẵn từ trước (gạt về OFF rồi bật lại)":
    "the ARM switch was already on (turn it OFF, then on again)",
  "cấu hình chưa lưu":
    "the settings are not saved",
  "la bàn chưa hiệu chuẩn":
    "the compass is not calibrated",
  "Bật motor.":
    "Starts the motors.",
  "Tự cân bằng, cần điều khiển góc nghiêng.":
    "Self-levelling; the sticks command the tilt angle.",
  "Tự cân bằng kiểu ArduPilot.":
    "Self-levelling, ArduPilot style.",
  "Giữ độ cao; tự bật Angle.":
    "Holds the height; switches Angle on by itself.",
  "Công tắc ép vào failsafe (hạ cánh).":
    "A switch that forces failsafe (descent).",
  "0° lật úp":
    "0° flipped",
  "90° lật úp":
    "90° flipped",
  "180° lật úp":
    "180° flipped",
  "270° lật úp":
    "270° flipped",
  "Không dùng":
    "Not used",
  "thành công":
    "done",
  "quá ít mẫu, xoay máy lâu hơn":
    "too few samples, keep turning longer",
  "chưa xoay đủ mọi hướng":
    "not turned in every direction yet",
  "dữ liệu không đủ để tính":
    "not enough data to compute",
  "dữ liệu không ra hình ellipsoid (có nhiễu từ mạnh?)":
    "the data does not form an ellipsoid (strong magnetic interference?)",
  "từ trường bị méo quá nhiều (để xa kim loại, dây nguồn)":
    "the field is distorted too much (keep away from metal and power wires)",
  "cường độ từ trường không hợp lý":
    "the field strength is not plausible",
  "khớp kém, làm lại":
    "poor fit, try again",
  "Đang ARM: firmware khóa mọi thay đổi. Disarm trước.":
    "ARMED: the firmware refuses every change. Disarm first.",
  "Chưa kết nối với flight controller.":
    "Not connected to a flight controller.",
  "Firmware không tìm thấy thiết bị cần cho việc này (xem tab Tổng quan).":
    "The firmware did not find the device this needs (see the Overview tab).",
  "Firmware chưa hỗ trợ việc này với cấu hình hiện tại.":
    "The firmware does not support this with the current settings.",
  "Firmware đang bận việc trước đó, thử lại sau một giây.":
    "The firmware is still busy with the previous request: try again in a second.",
  "Chưa đúng bước để làm việc này.":
    "This is not the right step for that yet.",
  "Firmware từ chối giá trị này.":
    "The firmware refused this value.",
  "Ghi flash thất bại.":
    "Writing to flash failed.",
  "Firmware báo thất bại.":
    "The firmware reports a failure.",
  "0 = tự nhận theo điện áp lúc cắm pin. Pin cắm vào lúc gần cạn bị nhận thiếu một cell (4S còn 12,8 V trông như 3S đầy): biết số cell thì nên nhập.":
    "0 = detected from the voltage when the pack is plugged in. A nearly empty pack is detected one cell short (a 4S at 12.8 V looks like a full 3S): enter the cell count if you know it.",
  "ADC đo pin: GPIO{pin} không đo được điện áp trên ESP32-S3 (chỉ GPIO1 đến GPIO20).":
    "Battery ADC: GPIO{pin} cannot measure a voltage on the ESP32-S3 (only GPIO1 to GPIO20).",
  "Accelerometer hoặc la bàn.":
    "Accelerometer or compass.",
  "Arm (không dùng GPS)":
    "Arm (GPS not in use)",
  "Arm, GPS chưa fix":
    "Arm, GPS has no fix",
  "Arm, GPS đã fix":
    "Arm, GPS has a fix",
  "Bật nguồn":
    "Power on",
  "Bật: pin dưới mức tối thiểu lúc đang bay thì firmware vào failsafe và hạ cánh. Tắt: chỉ có còi báo.":
    "On: a pack below the minimum in flight starts the failsafe landing. Off: the buzzer warns, nothing else.",
  "Chân ADC":
    "ADC pin",
  "Chân ADC chưa đo được điện áp nào. Cắm pin và kiểm tra chân \"ADC đo pin\" ở tab Chân và cổng.":
    "The ADC pin reads no voltage. Connect the battery and check the \"Battery ADC\" pin in the Pins and ports tab.",
  "Chưa có số đo pin":
    "No battery reading yet",
  "Chưa có điểm home.":
    "No home point yet.",
  "Chỉ cho còi thụ động.":
    "Passive buzzers only.",
  "Chờ trước khi báo nguy hiểm":
    "Delay before critical",
  "Chờ trước khi cảnh báo":
    "Delay before the warning",
  "Chủ động (tự kêu khi cấp điện)":
    "Active (sounds when powered)",
  "Các ngưỡng có hiệu lực ngay, không cần khởi động lại. Dưới mức cảnh báo thì firmware không cho arm.":
    "The limits apply at once, no reboot needed. Below the warning level the firmware refuses to arm.",
  "Còi":
    "Buzzer",
  "Công tắc Beeper":
    "Beeper switch",
  "Cảnh báo":
    "Warning",
  "Dùng để nhận số cell.":
    "Used to detect the cell count.",
  "ESC (DShot hai chiều)":
    "ESC (bidirectional DShot)",
  "Failsafe, sao lưu cấu hình và nhật ký của firmware.":
    "Failsafe, settings backup and the firmware log.",
  "GPS sẵn sàng":
    "GPS ready",
  "Gyro hiệu chuẩn xong":
    "Gyro calibrated",
  "Gạt công tắc ARM nhưng còn điều kiện chặn (xem thanh trên cùng).":
    "The ARM switch was flipped while something still blocks arming (see the top bar).",
  "Hiệu chuẩn accelerometer: xong một bước":
    "Accelerometer calibration: one step done",
  "Hiệu chuẩn thất bại":
    "Calibration failed",
  "Hiệu chỉnh số đo (ADC)":
    "Calibrate the reading (ADC)",
  "Hệ số chia mới: {k}. Số đọc sẽ khớp sau vài giây; bấm \"Lưu\" để giữ lại.":
    "New divider: {k}. The reading follows within a few seconds; press \"Save\" to keep it.",
  "Hệ số cầu chia áp":
    "Voltage divider ratio",
  "Im lặng":
    "Silence",
  "Khi nào thì kêu":
    "When it beeps",
  "Không arm được":
    "Arming refused",
  "Không có pin":
    "No battery",
  "Không đo":
    "Off",
  "Kêu cả khi đang nối app mà không có pin":
    "Beep also with the app connected and no battery",
  "Kêu khi bật mode Beeper trên tay điều khiển (tab Mode).":
    "Beeps while the Beeper mode is switched on from the transmitter (Modes tab).",
  "Kêu khi chân ở mức thấp":
    "Sounds when the pin is low",
  "Loại còi":
    "Buzzer type",
  "Lặp lại để tìm máy. Không kêu khi app đang kết nối.":
    "Repeats, to find the craft. Silent while the app is connected.",
  "Mất tín hiệu receiver":
    "Receiver signal lost",
  "Mẫu âm":
    "Pattern",
  "Mẫu âm dài {s} giây":
    "Sound pattern, {s} seconds long",
  "Mỗi cell":
    "Per cell",
  "Mỗi cell {v} V; cảnh báo dưới {warn} V, tối thiểu {min} V, đầy {max} V":
    "{v} V per cell; warning below {warn} V, minimum {min} V, full {max} V",
  "Một lần, khi GPS fix đủ vệ tinh lúc chưa arm.":
    "Once, when the GPS gets a fix with enough satellites while disarmed.",
  "Mức cảnh báo mỗi cell":
    "Warning level per cell",
  "Mức tối thiểu mỗi cell":
    "Minimum per cell",
  "Nguy hiểm":
    "Critical",
  "Nguồn đo điện áp":
    "Voltage source",
  "Ngưỡng":
    "Limits",
  "Nhập điện áp pin đo bằng đồng hồ (1 đến 60 V).":
    "Enter the battery voltage measured with a multimeter (1 to 60 V).",
  "Nhắc mỗi 2,6 giây rằng cánh quạt có thể quay.":
    "A reminder every 2.6 seconds that the propellers are live.",
  "Pin dưới mức cảnh báo":
    "Battery below the warning level",
  "Pin dưới mức tối thiểu":
    "Battery below the minimum",
  "Pin nối qua cầu chia áp vào một chân ADC. Đo điện áp pin bằng đồng hồ, nhập vào đây, app tính hệ số chia từ điện áp đang có ở chân.":
    "The battery reaches an ADC pin through a voltage divider. Measure the battery with a multimeter and enter it here: the app works out the divider from the voltage now at the pin.",
  "Pin và còi":
    "Battery and beeper",
  "Pin đang đo":
    "Battery now",
  "SOS bằng morse trong lúc máy tự hạ xuống.":
    "SOS in morse while the craft brings itself down.",
  "Sau khi bật nguồn, và khi hiệu chuẩn la bàn thành công.":
    "After power-on, and when a compass calibration succeeds.",
  "Số cell":
    "Cells",
  "Thấp hơn mức này: còi báo, không cho arm.":
    "Below this: the buzzer warns, arming is refused.",
  "Thấp hơn mức này: nguy hiểm, phải hạ cánh.":
    "Below this: critical, land now.",
  "Thụ động (cần xung PWM)":
    "Passive (needs a PWM tone)",
  "Thử":
    "Try",
  "Tình huống":
    "Situation",
  "Tính hệ số chia":
    "Work out the divider",
  "Tần số âm":
    "Tone",
  "Tắt (mặc định): im lặng trên bàn làm việc, chỉ nút \"Thử\" là kêu.":
    "Off (default): silent on the bench, only the \"Try\" button sounds.",
  "Tốt":
    "Good",
  "Tự hạ cánh khi pin nguy hiểm":
    "Land when the battery is critical",
  "Xoá xong bộ nhớ blackbox":
    "Blackbox memory erased",
  "chân ADC":
    "ADC pin",
  "chưa gán chân":
    "no pin assigned",
  "im lặng":
    "silent",
  "không có pin":
    "no battery",
  "pin dưới mức cảnh báo":
    "battery below the warning level",
  "ví dụ 16.42":
    "e.g. 16.42",
  "{v} V, {cells} cell, {cell} V mỗi cell":
    "{v} V, {cells} cells, {cell} V per cell",
  "Đang arm, ga ở thấp nhất":
    "Armed with the throttle down",
  "Đang hạ cánh failsafe":
    "Failsafe landing",
  "Điểm home đã được ghi.":
    "The home point is set.",
  "Điện áp":
    "Voltage",
  "Điện áp cell đầy":
    "Full cell voltage",
  "Điện áp pin chia cho điện áp ở chân ADC. Cầu 10k : 1k là 11.":
    "Battery voltage divided by the voltage at the ADC pin. A 10k : 1k divider is 11.",
  "Điện áp đo bằng đồng hồ":
    "Voltage measured with a multimeter",
  "Điện áp ở chân ADC":
    "Voltage at the ADC pin",
  "Điện áp ở chân ADC không được vượt 3,1 V khi pin đầy. Pin 6S (25,2 V) cần hệ số chia từ 8,2 trở lên.":
    "The ADC pin must stay below 3.1 V with a full pack. A 6S pack (25.2 V) needs a divider of 8.2 or more.",
  "Đo pin":
    "Battery sensing",
  "Đạp ga làm điện áp sụt trong chốc lát; chỉ tính khi thấp liên tục lâu hơn mức này.":
    "A throttle punch sags the pack for a moment; only a voltage that stays low for longer than this counts.",
  "Tự chọn (ADC nếu có chân, không thì ESC)":
    "Automatic (ADC pin if set, else ESC)",
  ", phiên thứ {n}":
    ", session {n}",
  ", thiếu {n} chỗ":
    ", {n} holes",
  "Bám theo dữ liệu mới":
    "Follow new data",
  "Bấm \"Xem\" ở một log trong danh sách trên, hoặc bắt đầu ghi trực tiếp.":
    "Press \"View\" on a log in the list above, or start a live recording.",
  "Bắt đầu ghi":
    "Start recording",
  "Bộ nhớ log trên mạch":
    "Log memory on the board",
  "Bộ nhớ log đã đầy. Tải log về máy rồi xoá để ghi tiếp.":
    "The log memory is full. Download the logs, then erase to record again.",
  "CSV dữ liệu chậm (pin, GPS, độ cao)":
    "CSV of the slow data (battery, GPS, altitude)",
  "Chưa có log nào. Tải log từ mạch về, ghi trực tiếp qua cáp, hoặc thêm một file .wblog có sẵn.":
    "No logs yet. Download one from the board, record live over the cable, or add a .wblog file you already have.",
  "Chưa có phiên nào trên mạch.":
    "No sessions on the board yet.",
  "Chưa kết nối flight controller: vẫn xem được các log đã lưu trên máy tính này.":
    "No flight controller connected: the logs saved on this computer can still be viewed.",
  "Chọn ít nhất một nhóm đồ thị ở trên.":
    "Choose at least one chart group above.",
  "Các lựa chọn này có hiệu lực ngay; bấm \"Lưu\" để giữ sau khi tắt nguồn. Phiên đang ghi dở giữ cấu hình lúc nó bắt đầu.":
    "These choices apply at once; press \"Save\" to keep them after power-off. A session that is running keeps the settings it started with.",
  "Cả pin":
    "Whole pack",
  "Cấu hình của chuyến bay này":
    "Settings of this flight",
  "Cần ga":
    "Throttle stick",
  "Cần điều khiển":
    "Sticks",
  "Dung lượng":
    "Size",
  "Dừng ghi":
    "Stop recording",
  "Firmware bỏ {n} bản ghi (hàng đợi đầy)":
    "The firmware dropped {n} records (queue full)",
  "Firmware gửi log thẳng về app thay vì ghi vào flash: xem đồ thị ngay lúc đang chạy, trên bàn hay khi bay có dây. Đang bay và đang ghi vào flash thì firmware từ chối, để không cắt ngang log của chuyến bay.":
    "The firmware sends the log straight to the app instead of writing it to flash: watch the charts while it runs, on the bench or on a tethered flight. While the craft is flying and logging to flash the firmware refuses, so the log of that flight is not cut short.",
  "Firmware không tìm thấy phân vùng flash dành cho log. Build firmware với một bảng phân vùng trong thư mục partitions/ (có phân vùng \"wblog\"), hoặc ghi trực tiếp qua cáp ở mục bên dưới.":
    "The firmware found no flash partition for the log. Build the firmware with one of the partition tables in partitions/ (they have a \"wblog\" partition), or record live over the cable below.",
  "Firmware làm tròn về một ước số của tần số vòng lặp. 250 là đủ để chỉnh PID; xem rung thì cần 1000 trở lên.":
    "The firmware rounds to a whole divisor of the loop rate. 250 is enough for PID tuning; looking at vibration needs 1000 or more.",
  "Firmware đã bỏ {n} bản ghi vì ghi không kịp. Giảm tốc độ ghi hoặc bỏ bớt nhóm.":
    "The firmware dropped {n} records because it could not write them in time. Lower the rate or record fewer groups.",
  "Flash trên mạch":
    "On-board flash",
  "Ga sau AltHold / failsafe":
    "Throttle after AltHold / failsafe",
  "Ghi gì, khi nào":
    "What to record, and when",
  "Ghi khi nào":
    "Record when",
  "Ghi log khi công tắc bật (tab Blackbox, mục \"Ghi khi nào\" chọn theo công tắc).":
    "Records the log while the switch is on (Blackbox tab, \"Record when\" set to the switch).",
  "Ghi lại chuyến bay (gyro, setpoint, PID, motor, pin, GPS) rồi xem bằng đồ thị; hoặc xem trực tiếp khi đang nối cáp.":
    "Record the flight (gyro, setpoint, PID, motors, battery, GPS) and look at it as charts; or watch it live over the cable.",
  "Ghi trực tiếp qua cáp":
    "Live recording over the cable",
  "Ghi vào":
    "Recorded to",
  "Ghi vào đâu":
    "Record to",
  "Gyro trước và sau lọc":
    "Gyro before and after filtering",
  "Gyro và setpoint":
    "Gyro and setpoint",
  "Góc nghiêng":
    "Attitude",
  "Hướng mũi (yaw)":
    "Heading (yaw)",
  "Khi bật công tắc Blackbox":
    "While the Blackbox switch is on",
  "Khoảng cách":
    "Distance",
  "Khoảng cách giữa hai vòng":
    "Time between two cycles",
  "Khoảng cách tới home":
    "Distance to home",
  "Khung receiver mỗi giây":
    "Receiver frames per second",
  "Khung/s":
    "Frames/s",
  "Không có bộ nhớ log":
    "No log memory",
  "Không ghi trên mạch":
    "Not on the board",
  "Kéo chuột để phóng to một đoạn, bấm đúp để xem cả log. Ctrl + lăn chuột: phóng to quanh con trỏ. Shift + kéo: dịch theo thời gian.":
    "Drag to zoom into a span, double-click for the whole log. Ctrl + wheel: zoom around the pointer. Shift + drag: move along the time axis.",
  "Kết thúc: {why}":
    "End: {why}",
  "Log này do một bản firmware có bộ cấu hình khác ghi ra: đồ thị vẫn đúng, nhưng phần cấu hình kèm theo không đọc được đầy đủ.":
    "This log was written by a firmware with a different settings layout: the charts are right, but the settings stored with it cannot be read completely.",
  "Log thiếu dữ liệu ở {n} chỗ (khung bị mất trên đường truyền, hoặc hỏng trong flash). Đồ thị để trống ở những chỗ đó.":
    "The log has {n} holes (frames lost on the link, or damaged in flash). The charts are left empty there.",
  "Log trên máy tính":
    "Logs on this computer",
  "Log được tải qua chính đường truyền này và lưu vào thư mục log trên máy tính. Mạch không hiện thành ổ USB.":
    "Logs are downloaded over this same link and saved into the log folder on the computer. The board does not show up as a USB drive.",
  "Lùi về trước":
    "Earlier",
  "Lúc":
    "Time",
  "Lúc lưu":
    "Saved",
  "Lưu file":
    "Save file",
  "Lệnh ra motor (0 đến 1)":
    "Motor outputs (0 to 1)",
  "MẤT {n}":
    "LOST {n}",
  "Nhóm đồ thị":
    "Chart groups",
  "PIN {state}":
    "BATTERY {state}",
  "Phiên":
    "Session",
  "Phiên này không có dấu kết thúc: mất nguồn khi đang ghi, hoặc file bị cắt. Dữ liệu trước đó vẫn dùng được.":
    "This session has no end marker: power was lost while recording, or the file was cut. The data before that is still good.",
  "Phiên trong file":
    "Session in the file",
  "Phiên {n} trong file ({t})":
    "Session {n} of the file ({t})",
  "Phân vùng log đang chứa dữ liệu khác hoặc log hỏng. Firmware không ghi đè lên đó: bấm \"Xoá bộ nhớ log\" trước.":
    "The log partition holds other data or a damaged log. The firmware does not write over it: press \"Erase log memory\" first.",
  "Phóng to":
    "Zoom in",
  "Phóng to quanh giữa (hoặc kéo chuột trên đồ thị)":
    "Zoom in around the middle (or drag on the chart)",
  "Phải xoá trước khi dùng":
    "Erase before use",
  "Pin: {state}":
    "Battery: {state}",
  "Qua cầu USB-UART ở {baud} baud, đường truyền chở được khoảng {n} bản ghi mỗi giây với các nhóm đang chọn (cổng USB gốc của ESP32-S3 nhanh hơn nhiều). Yêu cầu nhiều hơn thì log có chỗ thiếu.":
    "Through a USB-UART bridge at {baud} baud the link carries about {n} records per second with the chosen groups (the native USB port of the ESP32-S3 is much faster). Ask for more and the log gets holes.",
  "Sau lọc":
    "Filtered",
  "Suốt lúc arm":
    "Whenever armed",
  "Sẵn sàng":
    "Ready",
  "Số vệ tinh":
    "Satellites",
  "Sự kiện":
    "Event",
  "Sự kiện trong log":
    "Events in the log",
  "Thu nhỏ":
    "Zoom out",
  "Thêm file .wblog…":
    "Add a .wblog file…",
  "Thư mục log: {dir}":
    "Log folder: {dir}",
  "Thời gian chạy":
    "Run time",
  "Thời gian vòng lặp":
    "Loop timing",
  "Thời lượng":
    "Length",
  "Toàn bộ":
    "Whole log",
  "Trước lọc":
    "Unfiltered",
  "Tải log thất bại ({why}).":
    "The log download failed ({why}).",
  "Tải tất cả về máy":
    "Download all",
  "Tải về máy":
    "Download",
  "Tốc độ":
    "Speed",
  "Tốc độ GPS":
    "GPS speed",
  "Tốc độ ghi":
    "Rate",
  "Tốc độ lên xuống":
    "Climb rate",
  "Tốc độ motor":
    "Motor speed",
  "Tốc độ quay":
    "Rotation rate",
  "Tới về sau":
    "Later",
  "Từ lúc bật nguồn":
    "From power-on",
  "Vệ tinh":
    "Satellites",
  "Với lựa chọn này: {n} byte mỗi bản ghi, khoảng {rate} mỗi giây.":
    "With these choices: {n} bytes per record, about {rate} per second.",
  "Xem":
    "View",
  "Xem cả log (hoặc bấm đúp vào đồ thị)":
    "Show the whole log (or double-click the chart)",
  "Xoá":
    "Delete",
  "Xoá bộ nhớ log":
    "Erase log memory",
  "Xoá file {file} khỏi máy tính?":
    "Delete the file {file} from this computer?",
  "Xoá toàn bộ log trên mạch? Việc này mất từ vài giây đến vài phút; trong lúc đó không arm được.":
    "Erase every log on the board? This takes from seconds to minutes; the craft cannot be armed meanwhile.",
  "Xuất CSV":
    "Export CSV",
  "app, qua cáp":
    "the app, over the cable",
  "bản ghi/giây":
    "records/s",
  "flash trên mạch":
    "on-board flash",
  "{m} phút {s} giây":
    "{m} min {s} s",
  "{n} bản ghi, {t}":
    "{n} records, {t}",
  "{n} bản ghi/giây":
    "{n} records/s",
  "{s} giây":
    "{s} s",
  "{used} trên {cap} ({pct}%)":
    "{used} of {cap} ({pct}%)",
  "Đang ghi":
    "Recording",
  "Đang gửi về app":
    "Streaming to the app",
  "Đang tải: {done} / {total}":
    "Downloading: {done} / {total}",
  "Đang xoá":
    "Erasing",
  "Điện áp pin":
    "Battery voltage",
  "Đã tải log về: {file}":
    "Log downloaded: {file}",
  "Đã xoá xong bộ nhớ log.":
    "The log memory is erased.",
  "Đầy":
    "Full",
  "Đồ thị":
    "Charts",
  "Đồ thị: {file}":
    "Charts: {file}",
  "Đồ thị: đang ghi trực tiếp":
    "Charts: live recording",
  "Độ cao":
    "Altitude",
  "đang hiệu chuẩn, test motor hoặc xoá bộ nhớ log":
    "a calibration, a motor test or a log erase is running",
  "Ước lượng":
    "Estimate",
  "Gyro sau lọc":
    "Filtered gyro",
  "Tốc độ quay mà PID nhìn thấy.":
    "The rotation rate the PID sees.",
  "Gyro trước lọc":
    "Unfiltered gyro",
  "Để xem rung và đánh giá bộ lọc.":
    "For looking at vibration and judging the filters.",
  "Tốc độ quay mà cần điều khiển yêu cầu.":
    "The rotation rate the sticks ask for.",
  "Các thành phần PID":
    "PID terms",
  "P, I, D và feedforward của ba trục.":
    "P, I, D and feedforward of the three axes.",
  "Roll, pitch, yaw, ga.":
    "Roll, pitch, yaw, throttle.",
  "Lệnh ra motor":
    "Motor outputs",
  "Góc nghiêng và hướng mũi":
    "Attitude and heading",
  "RPM motor":
    "Motor RPM",
  "Cần DShot hai chiều.":
    "Needs bidirectional DShot.",
  "Thời gian chạy và khoảng cách giữa hai vòng.":
    "Run time and the time between two cycles.",
  "Barometer, lidar và ước lượng (10 lần mỗi giây).":
    "Barometer, lidar and the estimate (10 times per second).",
  "10 lần mỗi giây.":
    "10 times per second.",
  "hết điều kiện ghi (disarm hoặc tắt công tắc)":
    "the trigger went off (disarmed, or the switch turned off)",
  "bộ nhớ đầy":
    "memory full",
  "app dừng ghi":
    "the app stopped the recording",
  "app lấy luồng log về":
    "the app took the log stream",
  "Mode: {list}":
    "Modes: {list}",
  "con trỏ {t}":
    "pointer {t}",
  "ADC đo pin: chân nối với cầu chia áp, trên ESP32-S3 là GPIO1 đến GPIO20. ADC đo dòng: chân ra của cảm biến dòng (cũng GPIO1 đến GPIO20); để trống nếu không có cảm biến dòng. Còi: chân bật / tắt còi, hoặc phát xung cho còi thụ động; để trống nếu máy không gắn còi (motor kêu thay, xem tab Pin và còi).":
    "Battery ADC: the pin behind the voltage divider, GPIO1 to GPIO20 on an ESP32-S3. Current ADC: the output of the current sensor (also GPIO1 to GPIO20); leave it empty without a current sensor. Buzzer: the pin that switches the buzzer, or carries the tone of a passive one; leave it empty if the craft has no buzzer (the motors beep instead, see the Battery and beeper tab).",
  "Bảng chân":
    "Pin map",
  "Bật còi và cho motor kêu để tìm máy (chọn ở tab Pin và còi).":
    "Sounds the buzzer and beeps the motors to find the craft (chosen in the Battery and beeper tab).",
  "Bộ chân gợi ý cho ESP32-S3 DevKitC":
    "Suggested pins for an ESP32-S3 DevKitC",
  "Bộ đệm ghi thẻ đầy 100 %: thẻ quá chậm so với tốc độ ghi, đang mất bản ghi. Giảm \"Tốc độ ghi\" hoặc đổi thẻ.":
    "The card's write buffer is 100 % full: the card is too slow for this log rate and records are being lost. Lower \"Rate\" or use another card.",
  "Bộ đệm ghi thẻ: cao nhất {pct} % trong giây vừa qua.":
    "Card write buffer: peak {pct} % during the last second.",
  "CPU (nhân chạy vòng bay)":
    "CPU (the core that runs the flight loop)",
  "Chưa gán chân cho còi (tab Chân và cổng). Các ô dưới đây chỉ có tác dụng khi có còi.":
    "No buzzer pin is set (Pins and ports tab). The fields below only matter when there is a buzzer.",
  "Chưa gán chân còi: tiếng báo do motor phát (ESC dùng DShot). Motor chỉ kêu được khi chưa arm.":
    "No buzzer pin: the motors make the sounds (the ESCs use DShot). Motors can only beep while disarmed.",
  "Chỉ có còi: ESC đang dùng giao thức {p}, không nhận lệnh beacon. Muốn motor kêu thì chọn một giao thức DShot ở tab Motor.":
    "Buzzer only: the ESCs use {p}, which has no beacon command. For the motors to beep choose a DShot protocol in the Motors tab.",
  "Chỗ trống đủ cho khoảng {t}.":
    "The free space is enough for about {t}.",
  "Còi gắn trên mạch":
    "Buzzer on the board",
  "Còi đang kêu: ":
    "Buzzer playing: ",
  "Dây nối dài mà thẻ không nhận thì hạ xuống 4 đến 10.":
    "If the card is on long wires and is not found, lower this to 4 .. 10.",
  "ESC có 5 âm beacon, đánh số 1 đến 5. Bấm \"Thử\" để nghe.":
    "ESCs have 5 beacon tones, numbered 1 to 5. Press \"Try\" to hear it.",
  "ESC dùng chính motor để phát tiếng bíp, nên máy không cần còi. Mỗi tiếng dài khoảng nửa giây: motor phát đúng SỐ tiếng của mẫu âm (3 tiếng = gyro hiệu chuẩn xong, 2 tiếng = vừa disarm), không phát được nhịp nhanh như còi.":
    "The ESCs beep with the motors themselves, so the craft needs no buzzer. One tone takes about half a second: the motors play the NUMBER of beeps of a pattern (3 = gyro calibrated, 2 = just disarmed), not the quick rhythm a buzzer can.",
  "Firmware cũng kiểm tra lúc khởi động: hai chức năng trùng một chân thì không cho arm, và ghi rõ trong báo cáo ở tab Tổng quan.":
    "The firmware checks this at boot as well: with two functions on one pin it refuses to arm, and says which ones in the report on the Overview tab.",
  "Hai cột dấu chọn: còi gắn trên mạch, và motor (ESC làm motor rung thành tiếng bíp). Nút \"Thử\" phát âm đó một lần trên cả hai, kể cả khi nó đang bị tắt.":
    "Two columns of check boxes: the buzzer on the board, and the motors (the ESCs make them beep). \"Try\" plays that sound once on both, even when it is switched off.",
  "Không ghi trên mạch: app vẫn ghi trực tiếp qua cáp được. Thẻ SD không làm chậm vòng bay; flash trên mạch làm cả hai nhân dừng chừng 1 ms mỗi trang ghi.":
    "No log on the craft: the app can still record live over the cable. An SD card does not slow the flight loop; the on-board flash stops both cores for about 1 ms per page written.",
  "Kiểu SPI: đủ 4 chân. Kiểu SDMMC 1 bit: bỏ trống CS, ba chân còn lại là CLK, CMD, D0. Chọn kiểu nối ở tab Blackbox.":
    "SPI wiring: all 4 pins. SDMMC 1-bit wiring: leave CS empty, the other three are CLK, CMD, D0. The wiring type is chosen in the Blackbox tab.",
  "Lỗi":
    "Error",
  "Motor chỉ kêu khi chưa arm và sau khi disarm 1,2 giây. Lúc đang bay không có tiếng báo nào từ motor: pin yếu khi bay phải xem trên tay điều khiển hoặc gắn thêm còi.":
    "The motors only beep while disarmed, and not before 1.2 seconds after a disarm. In flight there is no sound from the motors at all: a low battery in flight has to be seen on the transmitter, or fit a buzzer.",
  "Motor kêu (DShot beacon)":
    "Motors as the beeper (DShot beacon)",
  "Motor đang quay (hoặc sắp quay) thì không kêu được":
    "A motor that is spinning (or about to) cannot beep",
  "Motor: {what}":
    "Motors: {what}",
  "Máy này chưa có gì để kêu: chưa gán chân còi, và ESC không dùng DShot nên motor không kêu được.":
    "This craft has nothing to make a sound with: no buzzer pin is set, and the ESCs do not use DShot, so the motors cannot beep.",
  "Máy này có còi và motor kêu được (ESC dùng DShot). Mỗi tình huống chọn riêng cho từng loại.":
    "This craft has a buzzer, and its motors can beep (the ESCs use DShot). Every condition is chosen separately for each.",
  "Một bộ chân không đụng chân strapping, USB, UART0, flash hay PSRAM của module N16R8: IMU 10-13, I2C 8/9, GPS 18/17, receiver 16/15, motor 4-7, ADC đo pin GPIO1, thẻ SD 39-42. Chỉ là điểm bắt đầu: dây nối trên mạch của bạn mới quyết định.":
    "A pin map that avoids the strapping pins, USB, UART0, the flash and the PSRAM of an N16R8 module: IMU 10-13, I2C 8/9, GPS 18/17, receiver 16/15, motors 4-7, battery ADC GPIO1, SD card 39-42. Only a starting point: the wiring of your board decides.",
  "PSRAM của module R8 (ví dụ N16R8): không dùng được":
    "the PSRAM of an R8 module (N16R8 for example): not usable",
  "SDMMC 1 bit (3 dây)":
    "SDMMC 1-bit (3 wires)",
  "SPI (4 dây)":
    "SPI (4 wires)",
  "SPI: 4 dây, hợp mọi module thẻ. SDMMC 1 bit: 3 dây, nhanh hơn, cần trở kéo lên ở CMD và D0, chân DAT3 của thẻ ở mức cao. Đổi kiểu nối: lưu rồi khởi động lại.":
    "SPI: 4 wires, works with every card module. SDMMC 1-bit: 3 wires, faster, needs pull-ups on CMD and D0 and the card's DAT3 pad held high. After changing the wiring type: save and restart.",
  "Thẻ SD":
    "SD card",
  "Thẻ SD (log chuyến bay)":
    "SD card (flight log)",
  "Thẻ SD kiểu SPI cần thêm chân CS (hoặc đổi sang kiểu SDMMC ở tab Blackbox).":
    "An SD card on SPI also needs the CS pin (or switch to SDMMC wiring in the Blackbox tab).",
  "Thẻ SD nối kiểu":
    "SD card wiring",
  "Thẻ SD đã đầy. Tải log về máy rồi xoá để ghi tiếp.":
    "The SD card is full. Download the logs, then erase to go on logging.",
  "Thẻ SD: cần đủ SCK / CLK, MOSI / CMD và MISO / D0.":
    "SD card: SCK / CLK, MOSI / CMD and MISO / D0 are all needed.",
  "Tiếng báo":
    "Sounds",
  "Tiếng báo (còi / motor)":
    "Beeper (buzzer / motors)",
  "Trạng thái, mode, pin và sóng receiver luôn được ghi 10 lần mỗi giây. Khi ESP32 ghi flash, cả hai nhân dừng chừng 1 ms cho mỗi trang; nhóm \"Thời gian vòng lặp\" ghi lại chính độ trễ đó để bạn đo trên mạch thật. Ghi ra thẻ SD thì không có độ trễ này.":
    "State, modes, battery and receiver link are always logged 10 times a second. When the ESP32 writes its flash, both cores stop for about 1 ms per page; the \"Loop timing\" group records exactly that delay, so you can measure it on the real board. Logging to an SD card does not have this delay.",
  "Tốc độ thẻ SD":
    "SD card speed",
  "Xoá mọi file log (wb_*.wblog trong thư mục /wblog) trên thẻ SD? Các file khác trên thẻ không bị đụng tới.":
    "Delete every log file (wb_*.wblog in the folder /wblog) on the SD card? Other files on the card are not touched.",
  "còn {free} trên {total} kB":
    "{free} of {total} kB free",
  "flash trên module có flash octal (N32R8V...)":
    "the flash of a module with octal flash (N32R8V...)",
  "hai chức năng đang dùng chung một chân GPIO (sửa ở tab Chân và cổng, lưu rồi khởi động lại)":
    "two functions share one GPIO (fix it in the Pins and ports tab, save and restart)",
  "motor đang phát: ":
    "motors playing: ",
  "motor: {n} tiếng":
    "motors: {n} tones",
  "{pct} % đang dùng, còn {free} kB":
    "{pct} % in use, {free} kB free",
  "Âm của motor":
    "Motor tone",
  "Đang không ghi trên mạch: chọn \"Flash trên mạch\" hoặc \"Thẻ SD\" ở mục \"Ghi vào đâu\".":
    "Nothing is being logged on the craft: choose \"On-board flash\" or \"SD card\" under \"Record to\".",
  "Điền bộ chân gợi ý":
    "Fill in the suggested pins",
  "Điền bộ chân gợi ý cho mạch ESP32-S3 DevKitC (module N16R8 hoặc N8) vào các ô? Những chân bạn đã nhập sẽ bị thay; chân còi và các chân khác giữ nguyên. Chưa có gì được lưu cho tới khi bạn bấm \"Lưu và khởi động lại\".":
    "Fill the boxes with the suggested pins for an ESP32-S3 DevKitC (N16R8 or N8 module)? Pins you already typed are replaced; the buzzer pin and the other pins stay as they are. Nothing is stored until you press \"Save and restart\".",
  "Điện áp pin, các ngưỡng cảnh báo, và tiếng báo (còi hoặc motor) cho từng tình huống.":
    "Battery voltage, the warning levels, and the sounds (buzzer or motors) for every condition.",
  "Đã điền bộ chân gợi ý. Đối chiếu với dây nối thật của mạch rồi mới lưu.":
    "Suggested pins filled in. Compare them with the real wiring of your board before saving.",
  "Chân của thẻ SD gán ở tab Chân và cổng (cần lưu và khởi động lại). File log nằm trong thư mục /wblog trên thẻ, tên wb_00001.wblog, wb_00002.wblog...: rút thẻ ra cắm vào máy tính rồi thêm bằng nút \"Thêm file .wblog…\" bên dưới, hoặc tải qua cáp như log trong flash.":
    "The SD card's pins are set in the Pins and ports tab (save and restart needed). The log files are in the folder /wblog on the card, named wb_00001.wblog, wb_00002.wblog...: put the card into the computer and add them with the \"Add a .wblog file…\" button below, or download them over the cable like the logs in flash.",
  "Chưa có file log nào trên thẻ.":
    "No log file on the card yet.",
  "Firmware không thấy thẻ SD (cứ 3 giây tìm lại một lần). Kiểm tra: thẻ đã cắm, định dạng FAT32 (không phải exFAT), đủ chân ở tab Chân và cổng, đúng kiểu nối; dây dài thì hạ \"Tốc độ thẻ SD\". Lý do firmware báo nằm ở tab Tổng quan.":
    "The firmware does not see a usable SD card (it looks again every 3 seconds). Check: card inserted, formatted FAT32 (not exFAT), all pins set in the Pins and ports tab, the right wiring type; on long wires lower \"SD card speed\". The reason the firmware gives is on the Overview tab.",
  "% của PID sum":
    "% of the PID sum",
  "% hành trình cần":
    "% of stick travel",
  "0 = tắt. Có RPM filter thì 1 đến 2 là đủ; không có thì 3 đến 5.":
    "0 = off. With the RPM filter 1 or 2 are enough; without it 3 to 5.",
  "0 = tắt. Mỗi motor một notch cho mỗi hài (1× , 2×, 3× tần số quay).":
    "0 = off. One notch per motor for each harmonic (1×, 2×, 3× the rotation frequency).",
  "0 đến 10. Cao hơn: cutoff tăng sớm hơn khi lên ga.":
    "0 to 10. Higher: the cutoff rises earlier as the throttle goes up.",
  "0 đến 95. Cao hơn: mượt hơn, trễ hơn.":
    "0 to 95. Higher: smoother, later.",
  "1.00 là bộ số mặc định của Betaflight, vốn dành cho quad 5 inch lực đẩy lớn. Máy nặng hoặc lực đẩy yếu thường cần Master Multiplier từ 1.3 đến 2.0; đổi từng nấc 0.1 và bay thử, sờ motor sau mỗi lần: motor nóng là D quá cao hoặc lọc quá ít.":
    "1.00 is Betaflight's default tune, which is made for a 5-inch quad with a lot of thrust. A heavy craft or one with little thrust usually needs a Master Multiplier of 1.3 to 2.0; change it 0.1 at a time, fly, and feel the motors after every flight: hot motors mean too much D or too little filtering.",
  "100: đang nghiêng mà gạt yaw thì máy lượn quanh trục thẳng đứng, mũi không chúi. 0: quay quanh trục của chính nó.":
    "100: yawing while tilted turns the craft about the vertical, the nose does not drop. 0: it turns about its own axis.",
  "15 cho quad 5 inch. Máy to, phản ứng chậm: giảm (10 hoặc thấp hơn).":
    "15 for a 5-inch quad. A big, slow craft: lower (10 or less).",
  "2 điểm":
    "2 point",
  "3 điểm":
    "3 point",
  "4 điểm":
    "4 point",
  "300 = Q 3. Cao hơn: notch hẹp hơn, ít trễ hơn, dễ trượt khỏi đỉnh nhiễu hơn.":
    "300 = Q 3. Higher: a narrower notch, less delay, easier to slip off the noise peak.",
  "ATC_ACCEL_R_MAX / P_MAX của ArduPilot. Thấp hơn: chuyển động mềm hơn.":
    "ArduPilot's ATC_ACCEL_R_MAX / P_MAX. Lower: softer movement.",
  "ATC_INPUT_TC: 0,10 gọn, 0,15 vừa, 0,30 rất mềm (quay phim).":
    "ATC_INPUT_TC: 0.10 crisp, 0.15 medium, 0.30 very soft (filming).",
  "Angle (Betaflight): góc nghiêng theo cần đi theo đường cong rate ở trên, nên nửa cần cho ít hơn nửa góc tối đa. Stabilize (ArduPilot): tỉ lệ thẳng với cần. Các hệ số của hai mode ở tab PID.":
    "Angle (Betaflight): the tilt for a stick position follows the rate curve above, so half stick gives less than half of the tilt limit. Stabilize (ArduPilot): proportional to the stick. The gains of both modes are in the PID tab.",
  "Angle: yaw theo trục thẳng đứng":
    "Angle: yaw about the vertical",
  "Bắt đầu giảm từ mức ga này.":
    "The reduction starts at this throttle.",
  "Bộ lọc có tác dụng ngay khi sửa (lúc disarm), và chỉ được giữ lại khi bấm \"Lưu\". Lọc nhiều hơn: motor mát hơn, máy trễ hơn. Lọc ít hơn: máy nhạy hơn, motor dễ nóng. Sau mỗi lần giảm lọc hãy bay thử 30 giây rồi sờ motor.":
    "Filters act as soon as they are edited (disarmed) and are kept only after \"Save\". More filtering: cooler motors, a later response. Less filtering: a sharper craft, motors that heat up easily. After every step towards less filtering fly for 30 seconds and feel the motors.",
  "Bộ lọc đang đặt ở đâu":
    "Where the filters sit",
  "Cho một cộng hưởng không đổi theo ga (khung, giá camera). Tìm tần số của nó bằng đồ thị phổ ở tab Blackbox.":
    "For a resonance that does not move with the throttle (frame, camera mount). Find its frequency with the spectrum chart in the Blackbox tab.",
  "Chỉ D":
    "D only",
  "Chỉnh sâu":
    "Expert",
  "Cutoff tĩnh":
    "Static cutoff",
  "Cutoff động, cao nhất":
    "Dynamic cutoff, highest",
  "Cutoff động, thấp nhất":
    "Dynamic cutoff, lowest",
  "Cả ba trục do thanh trượt tính: các ô trong bảng bên dưới bị khoá và hiện kết quả.":
    "All three axes are computed by the sliders: the boxes in the table below are locked and show the result.",
  "Cần nhích chậm hơn số bước này mỗi khung thì feedforward bị giảm: giấu run tay.":
    "A stick that creeps by fewer steps than this per frame gets less feedforward: hides finger tremor.",
  "Cắt feedforward khi setpoint sắp chạm tốc độ tối đa, để không vọt lố. 0 = tắt.":
    "Cuts feedforward as the setpoint approaches the top rate, to avoid overshoot. 0 = off.",
  "Cộng thêm phần ga thay đổi nhanh, để motor phản ứng kịp. 0 = tắt.":
    "Adds the fast-changing part of the throttle on top, so the motors react in time. 0 = off.",
  "Cộng thêm theo gia tốc của cần: đẩy motor vượt qua độ trễ lúc bắt đầu chuyển động.":
    "Adds the stick's acceleration: pushes the motors over their lag at the start of a move.",
  "D Max cao hơn D bao nhiêu. 0: D Max bằng D (không tăng D khi máy bị lắc).":
    "How far D Max sits above D. 0: D Max equals D (no extra D when the craft is shaken).",
  "D của mọi trục. Cao hơn: hãm rung và nẩy tốt hơn, motor nóng hơn.":
    "D of every axis. Higher: damps wobble and bounce better, hotter motors.",
  "D của pitch so với roll.":
    "Pitch D relative to roll.",
  "D tăng về phía D Max khi gyro dao động mạnh (prop wash). 0 = không theo gyro.":
    "D rises towards D Max when the gyro shakes hard (prop wash). 0 = not from the gyro.",
  "D tăng về phía D Max ngay khi cần chuyển động nhanh, trước cả khi máy kịp phản ứng.":
    "D rises towards D Max as soon as the stick moves fast, before the craft has even reacted.",
  "D-term notch và yaw lowpass":
    "D-term notch and yaw lowpass",
  "D-term notch, cutoff dưới":
    "D-term notch, lower cutoff",
  "D-term notch, tần số giữa":
    "D-term notch, centre frequency",
  "DShot hai chiều đang bật: RPM filter có số liệu để chạy.":
    "Bidirectional DShot is on: the RPM filter has the data it needs.",
  "Do thanh trượt tính":
    "Computed by the slider",
  "Dynamic lowpass trượt trong vùng xám nhạt theo ga (thấp khi ga thấp). Khung nét đứt: vùng dynamic notch được phép chạy. Vị trí \"lúc này\" chỉ có khi đang kết nối.":
    "A dynamic lowpass moves inside the light grey band with the throttle (low at low throttle). Dashed frame: the range the dynamic notches may move in. The \"right now\" positions are only there while connected.",
  "Dùng khi tắt chế độ động (cutoff động thấp nhất = 0). 0 = tắt.":
    "Used when dynamic mode is off (lowest dynamic cutoff = 0). 0 = off.",
  "Feedforward giảm dần về 0 khi cần ở gần giữa trong khoảng này. 0 = đủ ở mọi vị trí.":
    "Feedforward fades to 0 as the stick nears centre within this range. 0 = full everywhere.",
  "Feedforward lấy tốc độ gạt cần đẩy thẳng ra motor, nên máy quay cùng lúc với cần thay vì chờ sai lệch lớn lên. Tắt hẳn: đặt cột Feedforward trong bảng về 0.":
    "Feedforward sends the speed of the stick straight to the motors, so the craft turns with the stick instead of waiting for an error to build up. To switch it off: set the Feedforward column of the table to 0.",
  "Feedforward. Cao hơn: máy quay theo cần sớm hơn; quá cao thì vọt lố khi dừng cần.":
    "Feedforward. Higher: the craft follows the stick sooner; too high and it overshoots when the stick stops.",
  "Firmware vừa tự lưu phần \"{name}\" (level trim khi bay).":
    "The firmware has just saved the \"{name}\" section by itself (in-flight level trim).",
  "Gia tốc góc tối đa roll / pitch":
    "Maximum angular acceleration, roll / pitch",
  "Gia tốc góc tối đa yaw":
    "Maximum angular acceleration, yaw",
  "Giảm D lúc ga rất thấp (cánh quạt bám kém), cho tới khi ga vượt breakpoint thấp lần đầu.":
    "Less D at very low throttle (the props have little grip), until the throttle first passes the low breakpoint.",
  "Giới hạn I-term":
    "I-term limit",
  "Giới hạn công suất motor và dynamic idle (cần DShot hai chiều) ở tab Motor.":
    "Motor output limit and dynamic idle (needs bidirectional DShot) are in the Motor tab.",
  "Giữ máy treo yên một chỗ, bật công tắc ít nhất 2 giây rồi tắt, hạ cánh và disarm: tư thế lúc treo trở thành \"cân bằng\" mới và được tự lưu.":
    "Hold the craft in a steady hover, switch on for at least 2 seconds and off again, land and disarm: the hover attitude becomes the new \"level\" and is saved by itself.",
  "Gyro notch tĩnh":
    "Static gyro notches",
  "Góc nghiêng khi đẩy hết cần, cho cả Angle lẫn Stabilize.":
    "Tilt at full stick, for Angle and for Stabilize.",
  "Góc nghiêng tối đa":
    "Tilt limit",
  "Hằng số thời gian của cần":
    "Input time constant",
  "I không bao giờ vượt quá phần này của giới hạn PID sum (500, yaw 400).":
    "I never passes this share of the PID sum limit (500, yaw 400).",
  "I-term relax, kiểu":
    "I-term relax, type",
  "I-term relax, trục":
    "I-term relax, axes",
  "Khoảng mờ dần":
    "Fade range",
  "Kiểu":
    "Type",
  "Kéo sang trái: lọc nhiều hơn (cutoff thấp hơn). Sang phải: lọc ít hơn. Thanh trượt đang bật thì các ô cutoff của lowpass bị khoá và hiện kết quả; tắt thanh trượt để nhập tay. Notch, dynamic notch và RPM filter không thuộc thanh trượt.":
    "To the left: more filtering (lower cutoffs). To the right: less. While a slider is on, the lowpass cutoff boxes are locked and show the result; switch the slider off to type them. Notches, dynamic notches and the RPM filter are not part of the sliders.",
  "Level trim khi bay":
    "In-flight level trim",
  "Làm trơn bậc thang giữa hai khung của receiver. Nên để bật.":
    "Smooths the steps between two receiver frames. Leave it on.",
  "Làm trơn tần số motor":
    "Motor frequency smoothing",
  "Lúc này":
    "Right now",
  "Lọc riêng phần P của yaw. 0 = tắt.":
    "Filters the P part of yaw only. 0 = off.",
  "Lực đẩy thêm cho yaw trong lúc giữ cần. 0 = tắt.":
    "Extra push for yaw while the stick is held. 0 = off.",
  "Máy nghiêng theo ngay tốc độ gạt cần.":
    "The craft tilts with the speed of the stick at once.",
  "Máy vẫn trôi khi cần ở giữa? Gán mode \"Level trim khi bay\" cho một công tắc (tab Mode). Khi bay: giữ máy treo yên một chỗ bằng cần, bật công tắc ít nhất 2 giây rồi tắt, hạ cánh và disarm. Tư thế lúc treo yên thành \"cân bằng\" mới, hai số trim ở trên đổi theo và được tự lưu vào flash.":
    "Does the craft still drift with the sticks centred? Give the mode \"In-flight level trim\" a switch (Modes tab). In flight: hold the craft in a steady hover with the sticks, switch on for at least 2 seconds and off again, land and disarm. The attitude of that hover becomes the new \"level\", the two trim values above change with it and are written to flash by themselves.",
  "Nhân mọi hệ số. Máy nặng, lực đẩy yếu: tăng lên. Máy nhỏ nhẹ rung nhanh: giảm xuống.":
    "Multiplies every gain. A heavy craft with little thrust: raise it. A small light one that oscillates fast: lower it.",
  "Notch 1, cutoff dưới":
    "Notch 1, lower cutoff",
  "Notch 1, tần số giữa":
    "Notch 1, centre frequency",
  "Notch 2, cutoff dưới":
    "Notch 2, lower cutoff",
  "Notch 2, tần số giữa":
    "Notch 2, centre frequency",
  "Notch yếu dần trong khoảng này phía trên tần số thấp nhất.":
    "The notch fades out over this range above the lowest frequency.",
  "P và D":
    "P and D",
  "P và I. Cao hơn: bám cần sát hơn; quá cao thì rung nhanh.":
    "P and I. Higher: follows the stick more tightly; too high and it oscillates fast.",
  "P, I và feedforward của pitch so với roll.":
    "Pitch P, I and feedforward relative to roll.",
  "PID có tác dụng ngay khi sửa (kể cả lúc máy đang nối USB trên bàn), nhưng chỉ được giữ lại khi bấm \"Lưu\". Đơn vị và mặc định là của Betaflight: số từ một bản tune Betaflight dùng được thẳng ở đây.":
    "PID settings act as soon as they are edited (also with the craft on the bench over USB), but are kept only after \"Save\". Units and defaults are Betaflight's: the numbers of a Betaflight tune can be used here as they are.",
  "PID sum: 1000 là toàn bộ dải motor; giới hạn 500 (roll, pitch) và 400 (yaw). Số nhảy quanh giới hạn khi bay bình thường là hệ số quá cao hoặc motor quá yếu.":
    "PID sum: 1000 is the whole motor range; the limits are 500 (roll, pitch) and 400 (yaw). A sum that keeps hitting the limit in normal flight means gains that are too high or motors that are too weak.",
  "Phải thấp hơn tần số giữa. Càng gần tần số giữa, notch càng hẹp.":
    "Must be below the centre frequency. The closer to it, the narrower the notch.",
  "Phần trăm tác dụng của notch ở hài 1, 2, 3. Cánh 2 lá thường chỉ cần hài 1 và 2; đặt hài 3 về 0 để bớt trễ.":
    "How much of the notch at harmonic 1, 2, 3 is applied. Two-blade props usually need only harmonics 1 and 2; set harmonic 3 to 0 for less delay.",
  "RPM filter cần DShot hai chiều (tab Motor). Hiện đang tắt, nên các notch này không có tác dụng; dynamic notch và lowpass gánh toàn bộ.":
    "The RPM filter needs bidirectional DShot (Motor tab). It is off, so these notches do nothing; the dynamic notches and the lowpasses carry the whole load.",
  "RPM notch (hài 1) từng motor":
    "RPM notch (harmonic 1) of each motor",
  "Rate và đường cong ga có tác dụng ngay khi sửa, nhưng chỉ được giữ lại khi bấm \"Lưu\".":
    "Rates and the throttle curve act as soon as they are edited, but are kept only after \"Save\".",
  "Receiver chỉ gửi vị trí cần vài chục đến vài trăm lần mỗi giây; firmware nối các bậc đó thành đường cong trơn trước khi đưa vào PID. Tần số cắt tự theo tốc độ khung đo được. Có hiệu lực sau khi lưu và khởi động lại.":
    "A receiver sends the stick positions only some tens to hundreds of times per second; the firmware joins those steps into a smooth curve before the PID sees them. The cutoff follows the measured frame rate by itself. Takes effect after saving and restarting.",
  "Riêng I. Cao hơn: giữ hướng tốt hơn khi đổi ga; quá cao thì lắc chậm.":
    "I only. Higher: holds its attitude better through throttle changes; too high and it wobbles slowly.",
  "Roll và pitch":
    "Roll and pitch",
  "Roll và pitch do thanh trượt tính; yaw nhập tay.":
    "Roll and pitch are computed by the sliders; yaw is typed in.",
  "Roll, pitch (chỉ chặn tăng)":
    "Roll, pitch (increase only)",
  "Roll, pitch và yaw":
    "Roll, pitch and yaw",
  "Roll, pitch, yaw (chỉ chặn tăng)":
    "Roll, pitch, yaw (increase only)",
  "Stabilize (kiểu ArduPilot)":
    "Stabilize (ArduPilot style)",
  "Stabilize P: tốc độ quay yêu cầu (°/s) trên mỗi độ sai lệch góc, riêng từng trục (ATC_ANG_RLL_P, ATC_ANG_PIT_P, ATC_ANG_YAW_P; mặc định 4,5). Yaw trong Stabilize giữ hướng mũi khi cần yaw ở giữa.":
    "Stabilize P: commanded rate (°/s) per degree of angle error, for each axis (ATC_ANG_RLL_P, ATC_ANG_PIT_P, ATC_ANG_YAW_P; default 4.5). In Stabilize yaw holds the heading while the yaw stick is centred.",
  "Số hài":
    "Harmonics",
  "Số notch mỗi trục":
    "Notches per axis",
  "TPA ga thấp luôn có tác dụng":
    "Low-throttle TPA always active",
  "TPA ga thấp, breakpoint":
    "Low-throttle TPA, breakpoint",
  "TPA ga thấp, rate":
    "Low-throttle TPA, rate",
  "TPA giảm":
    "TPA reduces",
  "TPA và ga":
    "TPA and throttle",
  "Thang của Betaflight: P, I, D, D Max từ 0 đến 250, Feedforward từ 0 đến 1000. D Max lớn hơn D thì D tự tăng về phía D Max khi máy bị lắc hoặc cần gạt nhanh (Dynamic Damping).":
    "Betaflight's scale: P, I, D, D Max from 0 to 250, Feedforward from 0 to 1000. With D Max above D, D rises towards D Max by itself when the craft is shaken or the stick moves fast (Dynamic Damping).",
  "Thanh trượt":
    "Sliders",
  "Thanh trượt cho bộ lọc D-term":
    "Slider for the D-term filters",
  "Thanh trượt cho bộ lọc gyro":
    "Slider for the gyro filters",
  "Thanh trượt tune":
    "Tuning sliders",
  "Thanh trượt đang tắt: mọi ô trong bảng bên dưới nhập tay.":
    "The sliders are off: every box in the table below is typed in.",
  "Thanh trượt điều khiển":
    "Sliders control",
  "Trung bình vài khung gần nhất: đường truyền nhiễu thì tăng.":
    "Averages the last few frames: raise it for a noisy link.",
  "Trọng số hài":
    "Harmonic weights",
  "Trục nào do thanh trượt tính thì ô P / I / D / D Max / F của trục đó bị khoá: firmware tự tính từ thanh trượt.":
    "The P / I / D / D Max / F boxes of an axis under the sliders are locked: the firmware computes them from the sliders.",
  "Tăng I (và P) trong lúc ga thay đổi nhanh để mũi không gật.":
    "Raises I (and P) while the throttle changes fast, so the nose does not nod.",
  "Tạm ngừng tích I khi cần đang chuyển động nhanh, để không nẩy lại sau cú lật. Loại \"_INC\": I vẫn được giảm.":
    "Stops the I-term from building while the stick moves fast, so it does not bounce back after a flip. The \"increase only\" kinds: I may still come down.",
  "Tần số cao nhất":
    "Highest frequency",
  "Tần số cắt = tốc độ khung × 1,5 / (1 + số này / 10), không dưới 15 Hz. Số lớn hơn: mượt hơn, trễ hơn.":
    "Cutoff = frame rate × 1.5 / (1 + this number / 10), never below 15 Hz. Larger: smoother, later.",
  "Tần số thấp nhất":
    "Lowest frequency",
  "Tắt (nhập tay mọi trục)":
    "Off (type every axis)",
  "Tắt: chỉ có tác dụng trước khi cất cánh.":
    "Off: only acts before take-off.",
  "Tắt: góc mục tiêu nhảy thẳng theo cần.":
    "Off: the target angle jumps with the stick.",
  "Tốc độ quay yêu cầu = số này / 10 × sai lệch góc (°).":
    "Commanded rate = this number / 10 × angle error (°).",
  "Tự tìm các đỉnh nhiễu còn lại trong gyro (cộng hưởng khung, nhiễu mà RPM filter không thấy) và đặt notch lên đó. Cần vòng lặp từ 2000 Hz.":
    "Finds the noise peaks that are left in the gyro (frame resonances, noise the RPM filter cannot see) and puts a notch on each. Needs a loop of 2000 Hz or faster.",
  "Vị trí của các bộ lọc trên trục tần số":
    "Where the filters sit on the frequency axis",
  "chưa đo":
    "not measured",
  "cutoff của lowpass (vùng xám bên phải bị lọc)":
    "lowpass cutoff (the grey area to its right is filtered)",
  "notch tĩnh":
    "static notch",
  "tắt":
    "off",
  "× 0,01":
    "× 0.01",
  "Đưa mọi thanh trượt về 1.00":
    "All sliders back to 1.00",
  "Định hình chuyển động (rate feedforward)":
    "Shape the movement (rate feedforward)",
  "Độ cong theo ga":
    "Curve over the throttle",
  "Độ làm trơn cần ga":
    "Smoothing of the throttle stick",
  "Độ làm trơn cần roll / pitch / yaw":
    "Smoothing of roll / pitch / yaw",
  "đang đo: {n} %":
    "measuring: {n} %",
  "đã đo xong, sẽ áp dụng khi disarm":
    "measured, will be applied at the disarm",
  "Ở ga 0. Đặt 0 để tắt chế độ động.":
    "At zero throttle. Set 0 to switch dynamic mode off.",
  "Ở ga tối đa, D (và P nếu chọn PD) bị giảm chừng này.":
    "At full throttle D (and P with mode PD) is reduced by this much.",
  "Ở ga tối đa.":
    "At full throttle.",
  "Ga treo đo được":
    "Measured hover throttle",
  "Ga treo: firmware tự đo trong mọi mode bay (trước chuyến bay đầu là số đoán). Altitude Hold và failsafe dùng số này, không cần nhập.":
    "Hover throttle: the firmware measures it in every flight mode (a guess before the first flight). Altitude Hold and the failsafe use it; there is nothing to enter.",
  "Cần ga lên tới mức này thì máy được coi là đang bay ngay; dưới mức này thì sau 1 giây có ga.":
    "At this throttle the craft counts as flying at once; below it, after 1 second of any throttle.",
  "Cần ga ở min mà máy vẫn được giữ ổn định. \"Khi đang bay\": tự tắt sau 1 giây máy đứng yên trên mặt đất (motor về idle, I-term về 0). \"Luôn bật\" là cách của Betaflight: trên bàn hoặc sau khi đáp, motor sẽ tự tăng tốc dần cho tới khi disarm.":
    "The craft stays stabilised with the throttle stick at the bottom. \"While flying\": ends after the craft has stood still on the ground for 1 second (motors to idle, I-terms to 0). \"Always\" is Betaflight's way: on the bench or after landing the motors run up by themselves until the disarm.",
  "Khi đang bay (tự tắt lúc đứng trên mặt đất)":
    "While flying (ends when standing on the ground)",
  "Luôn bật tới khi disarm (Betaflight)":
    "Always, until the disarm (Betaflight)",
  "Mức ga tính là đã cất cánh":
    "Throttle that counts as airborne",
  "Tắt (cần ga ở min = motor idle)":
    "Off (stick at the bottom = motors at idle)",
  "trên mặt đất: motor ở idle":
    "on the ground: motors at idle",
  "đang bay":
    "flying",
  "đang bay, cần ga ở min (airmode)":
    "flying, throttle stick at the bottom (airmode)",
  " theo cấu hình, thực tế {n}":
    " configured, {n} in the log",
  "\"Khung bỏ qua\": vòng điều khiển gọi nhanh hơn một khung DShot kèm lượt trả lời có thể đi hết (DShot300 hai chiều cần 182 µs, DShot600 cần 116 µs) - hạ tần số vòng hoặc dùng DShot nhanh hơn. \"Lượt trả lời bị lỡ\": firmware đổi chiều chân tín hiệu không kịp.":
    "\"Frames skipped\": the control loop asks for frames faster than one DShot frame with its reply can pass (bidirectional DShot300 needs 182 µs, DShot600 needs 116 µs) - lower the loop rate or use a faster DShot. \"Replies missed\": the firmware did not turn the signal pin around in time.",
  "\"Thời gian từng khâu\" là trung bình của giây vừa rồi cho một chu kỳ điều khiển; cộng lại phải nhỏ hơn 1 000 000 / tần số đã đặt (250 µs ở 4000 Hz). Khâu nào lớn bất thường là chỗ cần xem: đọc IMU (dây SPI, tốc độ SPI), tính toán (bộ lọc, RPM filter), gửi motor (DShot); \"còn lại\" gồm chờ luồng cảm biến nhả khoá, ghi log và telemetry. Số lớn nhất của từng khâu nằm trong log blackbox (nhóm \"Thời gian vòng lặp\").":
    "\"Time per stage\" is the average of the last second for one control cycle; the sum has to stay below 1 000 000 / the configured rate (250 µs at 4000 Hz). A stage that is unusually large is where to look: IMU read (SPI wiring, SPI speed), computing (filters, RPM filter), motor output (DShot). \"The rest\" is waiting for the sensor thread to release the lock, logging and telemetry. The largest value of each stage is in the blackbox log (group \"Loop timing\").",
  ", ESC không trả lời: {list}":
    ", ESCs not answering: {list}",
  ", mọi ESC đều trả lời":
    ", every ESC answers",
  "0 = không phát xung. Nhiều ESC analog (PWM, Oneshot, Multishot) chỉ khởi động khi thấy xung thấp nhất: nếu ESC không kêu tiếng sẵn sàng, đặt 1000.":
    "0 = no pulse. Many analog ESCs (PWM, Oneshot, Multishot) only start up when they see the minimum pulse: if the ESC does not play its ready tone, set 1000.",
  "Bỏ cuộc sau khi không xuống được":
    "Give up when it does not come down for",
  "Chặn runaway lúc cất cánh":
    "Runaway take-off prevention",
  "Chỉ đếm lúc máy KHÔNG đi xuống theo ước lượng độ cao (mắc vào đâu đó), hoặc khi không có ước lượng độ cao. Hết thời gian thì tắt motor. Một lần hạ cánh từ trên cao không bao giờ bị cắt giữa chừng.":
    "Counted only while the craft is NOT coming down by its height estimate (caught on something), or when there is no height estimate. When the time is up the motors stop. A landing from high up is never cut short on the way.",
  "Cùng ý nghĩa với \"board alignment\" của Betaflight: mạch có target Betaflight thì chép đúng giá trị của target đó. Cách kiểm tra: hình 3D ở tab Tổng quan phải nghiêng đúng như máy trên tay.":
    "Same meaning as Betaflight's \"board alignment\": for a board with a Betaflight target, copy the value of that target. To check: the 3D view on the Overview tab must tilt exactly like the craft in your hand.",
  "DShot hai chiều: ":
    "Bidirectional DShot: ",
  "ESC phải chạy firmware có bidirectional DShot (BLHeli_32 từ 32.7, Bluejay, AM32). Dòng trạng thái dưới bảng motor cho biết từng ESC có trả lời không.":
    "The ESC must run firmware with bidirectional DShot (BLHeli_32 from 32.7, Bluejay, AM32). The status line under the motor table tells whether each ESC answers.",
  "Failsafe và an toàn":
    "Failsafe and safety",
  "Khung DShot bỏ qua trong giây vừa rồi: ":
    "DShot frames skipped in the last second: ",
  "Log thưa hơn cấu hình: vòng điều khiển lúc đó chạy chậm hơn tần số đã đặt, hoặc (khi ghi qua cáp) firmware đã giảm tốc độ ghi cho vừa đường truyền.":
    "The log is sparser than configured: the control loop ran below its configured rate at the time, or (recording over the cable) the firmware lowered the rate to fit the link.",
  "Luồng cảm biến, lượt dài nhất":
    "Sensor thread, longest pass",
  "Lượt trả lời bị lỡ: ":
    "Replies missed: ",
  "Mất sóng quá thời gian chờ thì máy tự cân bằng và hạ xuống; chạm đất thì tắt motor. Sóng trở lại và giữ ổn định thì bạn lấy lại quyền điều khiển; công tắc Failsafe thì hết ngay khi tắt công tắc.":
    "With the link lost longer than the guard time the craft levels itself and comes down; on the ground the motors stop. When the link is back and stays, you get control back; the Failsafe switch ends the moment it is switched off.",
  "Pin dưới mức tối thiểu cũng làm máy tự hạ xuống (tab Pin và còi), nhưng roll, pitch và yaw vẫn theo cần của bạn để chọn chỗ đáp; lần hạ cánh này không huỷ được. Đang đứng trên mặt đất mà mất sóng thì motor tắt luôn.":
    "A battery below its minimum also brings the craft down (Battery and beeper tab), but roll, pitch and yaw still follow your sticks so you can choose where it lands; this landing cannot be called off. Standing on the ground when the link is lost, the motors simply stop.",
  "Sai thứ tự motor, lắp ngược cánh, motor quay sai chiều hay sai hướng chip IMU làm PID đẩy máy lật thay vì giữ thăng bằng. Trước khi máy bay bình thường được nửa giây (ga từ 20 % kèm một cần đã gạt, hoặc ga từ 40 %): PID dồn hết lực mà máy vẫn quay thì motor tắt sau 75 ms. Giống Betaflight. Nên để bật.":
    "A wrong motor order, a prop on upside down, a motor turning the wrong way or a wrong IMU orientation make the PID tip the craft over instead of balancing it. Until the craft has flown normally for half a second (throttle from 20 % with a stick moved, or throttle from 40 %): when the PID pushes with all it has and the craft still turns, the motors stop after 75 ms. Same as Betaflight. Leave it on.",
  "Thời gian từng khâu":
    "Time per stage",
  "công tắc Failsafe":
    "Failsafe switch",
  "không":
    "none",
  "motor vừa bị tắt vì runaway lúc cất cánh: kiểm tra thứ tự motor, chiều quay, chiều lắp cánh và hướng chip IMU (gạt ARM về OFF để xoá)":
    "the motors were just stopped for a runaway take-off: check motor order, spin direction, how the props are mounted and the IMU orientation (switch ARM off to clear)",
  "pin dưới mức tối thiểu":
    "battery below its minimum",
  "vòng điều khiển không đạt tần số đã đặt (xem \"Thời gian từng khâu\" ở tab Tổng quan; hạ \"Tần số vòng gyro / PID\" nếu cần)":
    "the control loop does not reach its configured rate (see \"Time per stage\" on the Overview tab; lower \"Gyro / PID loop rate\" if needed)",
  "{hz} Hz (đặt {target} Hz), bỏ lỡ {n} chu kỳ trong giây vừa rồi":
    "{hz} Hz ({target} Hz configured), {n} cycles missed in the last second",
  "Đang tự hạ cánh mà sóng trở lại liên tục chừng này thì máy trả quyền điều khiển cho bạn.":
    "During the automatic landing, a link that is back for this long without a break hands control back to you.",
  "đang chạy":
    "running",
  "đang chờ":
    "waiting",
  "đang tự hạ cánh":
    "landing by itself",
  "đã bật nhưng không chạy (driver IDF chỉ một chiều, hoặc kiểm tra lúc khởi động không đạt: xem dòng motors ở tab Tổng quan); motor chạy DShot một chiều":
    "on, but not running (the IDF driver is one-way only, or the start-up check failed: see the motors line on the Overview tab); the motors run one-way DShot",
  "đã hạ cánh, motor tắt":
    "landed, motors off",
  "đọc IMU {imu} · tính toán {core} · gửi motor {motor} · còn lại {rest} µs":
    "IMU read {imu} · computing {core} · motor output {motor} · the rest {rest} µs",
  "Chờ khoá":
    "Waiting for the lock",
  "Gửi motor":
    "Motor output",
  "Tính toán, chu kỳ có bộ ước lượng":
    "Computation, cycle with the estimators",
  "Tính toán, chu kỳ thường":
    "Computation, plain cycle",
  "Từng khâu của một chu kỳ (lớn nhất giữa hai bản ghi)":
    "The stages of a cycle (longest between two records)",
  "Đọc IMU":
    "IMU read",
  "Vận tốc trên mặt đất":
    "Speed over the ground",
  "Ước lượng, bắc":
    "Estimate, north",
  "Ước lượng, đông":
    "Estimate, east",
  "Optical flow, tiến":
    "Optical flow, forwards",
  "Optical flow, sang phải":
    "Optical flow, to the right",
  "Vị trí và điểm đang giữ":
    "Position and the held point",
  "Vị trí, bắc":
    "Position, north",
  "Vị trí, đông":
    "Position, east",
  "Điểm giữ, bắc":
    "Held point, north",
  "Điểm giữ, đông":
    "Held point, east",
  "Góc nghiêng autopilot yêu cầu":
    "Lean asked for by the autopilot",
  "Chất lượng bề mặt (optical flow)":
    "Surface quality (optical flow)",
  "Chất lượng":
    "Quality",
  "Altitude Hold, Position Hold và GPS Rescue tự bật Angle. Mode không có dải nào thì luôn tắt. Thay đổi ở tab này có tác dụng ngay, nhưng vẫn cần bấm \"Lưu\" để giữ lại sau khi tắt nguồn.":
    "Altitude Hold, Position Hold and GPS Rescue switch Angle on by themselves. A mode without a range is always off. Changes on this tab take effect at once, but \"Save\" is still needed to keep them after power-off.",
  "mũi máy":
    "nose",
  "phải":
    "right",
  "Tốc độ optical flow đang đo: mũi tên chỉ hướng máy đang đi, mũi máy ở phía trên":
    "Speed measured by optical flow: the arrow points where the craft is going, its nose is at the top",
  "firmware chưa khởi động lại từ khi chọn cảm biến":
    "the firmware has not been restarted since the sensor was chosen",
  "chưa chọn cảm biến":
    "no sensor chosen",
  "Cảm biến optical flow, ước lượng vị trí, giữ vị trí (Position Hold) và tự bay về (GPS Rescue).":
    "Optical flow sensor, position estimate, Position Hold and flying home by itself (GPS Rescue).",
  "Các chức năng ở tab này mới được kiểm chứng bằng mô phỏng trên máy tính, chưa bay thật. Lần đầu thử: bay thấp, chỗ rộng, tay sẵn sàng tắt công tắc Position Hold / Rescue để lấy lại quyền điều khiển.":
    "The functions on this tab have only been verified in simulation on a PC, not in real flight. The first time: fly low, with room around you, and keep a finger on the Position Hold / Rescue switch to take the craft back.",
  "Cảm biến optical flow":
    "Optical flow sensor",
  "Loại cảm biến và baud có hiệu lực sau khi lưu và khởi động lại. Chân nối: tab Chân và cổng, nhóm \"Optical flow\".":
    "Sensor type and baud rate take effect after saving and restarting. Wiring: tab Pins and ports, group \"Optical flow\".",
  "Số mẫu nhận được":
    "Samples received",
  "{n} mẫu/giây":
    "{n} samples/s",
  "Bề mặt bên dưới":
    "Surface below",
  "chất lượng {q} / 100":
    "quality {q} / 100",
  "Mẫu mới nhất":
    "Latest sample",
  "Tốc độ trên mặt đất":
    "Speed over the ground",
  "tiến {fwd} m/s, sang phải {right} m/s":
    "forwards {fwd} m/s, to the right {right} m/s",
  "Kiểm tra hướng lắp: cầm máy cách sàn 0,5 đến 1 m (sàn có hoa văn, đủ sáng), giữ phẳng. Đưa máy về phía mũi: mũi tên phải chỉ lên. Đưa sang phải: mũi tên chỉ sang phải. Mũi tên lệch một phần tư hay nửa vòng: đổi \"Hướng lắp cảm biến\". Trái phải ngược nhau: bật \"Ảnh bị lật gương\". Chỉ lắc máy tại chỗ thì mũi tên phải gần như đứng yên.":
    "Checking the mounting direction: hold the craft level, 0.5 to 1 m above a floor with some pattern and enough light. Move it towards its nose: the arrow must point up. Move it to the right: the arrow must point right. Arrow off by a quarter or half a turn: change \"Sensor mounting direction\". Left and right exchanged: switch on \"Mirrored picture\". Rocking the craft on the spot must leave the arrow almost still.",
  "Góc nghiêng ước lượng":
    "Tilt estimate",
  "Optical flow PMW3901: cần đủ bốn chân CS, SCK, MISO, MOSI.":
    "Optical flow PMW3901: all four pins CS, SCK, MISO, MOSI are needed.",
  "Optical flow qua UART: chưa có chân RX.":
    "Optical flow on a UART: the RX pin is not set.",
  "PMW3901 và thẻ SD kiểu SPI cần cùng một bus SPI: đổi thẻ sang SDMMC (tab Blackbox) hoặc dùng cảm biến optical flow qua UART.":
    "PMW3901 and an SD card in SPI mode need the same SPI bus: put the card on SDMMC (Blackbox tab) or use an optical flow sensor on a UART.",
  "cảm biến không gửi mẫu nào":
    "the sensor sends no samples",
  "Ước lượng vị trí":
    "Position estimate",
  "Vận tốc lấy từ":
    "Velocity comes from",
  "chưa có":
    "nothing yet",
  "Vận tốc (bắc / đông)":
    "Velocity (north / east)",
  "Vị trí so với home (bắc / đông)":
    "Position relative to home (north / east)",
  "chưa có (cần GPS fix)":
    "none (needs a GPS fix)",
  "Hướng mũi":
    "Heading",
  "theo la bàn":
    "from the compass",
  "chỉ theo gyro (không biết hướng bắc)":
    "gyro only (north is unknown)",
  "đang được chỉnh theo gia tốc đo từ GPS / optical flow":
    "being corrected from the acceleration measured by GPS / optical flow",
  "chỉ theo accelerometer":
    "accelerometer only",
  "Buông cần roll / pitch thì máy tự phanh rồi đứng tại chỗ, chống lại gió; đẩy cần thì bay như Angle. Cần một trong hai: optical flow + lidar (trong nhà, bay thấp), hoặc GPS đã fix + la bàn đã hiệu chuẩn. Gán công tắc ở tab Mode và nên bật cùng Altitude Hold.":
    "Let go of the roll / pitch stick and the craft brakes and then stays where it is, against the wind; push the stick and it flies as in Angle. It needs one of two: optical flow + lidar (indoors, flying low), or a GPS fix + a calibrated compass. Assign a switch on the Modes tab; best used together with Altitude Hold.",
  "đang giữ vị trí":
    "holding position",
  "công tắc đang bật nhưng chưa có gì để giữ vị trí":
    "the switch is on but there is nothing to hold a position with",
  "Góc nghiêng autopilot yêu cầu (roll / pitch)":
    "Lean asked for by the autopilot (roll / pitch)",
  "Điểm đang giữ (bắc / đông)":
    "Held point (north / east)",
  "Bù gió đã học (bắc / đông)":
    "Wind compensation learned (north / east)",
  "GPS Rescue và failsafe":
    "GPS Rescue and failsafe",
  "Rescue: máy leo lên độ cao bay về, quay mũi về điểm cất cánh, bay về, hạ dần rồi hạ cánh và tắt motor. Chạy khi bật công tắc GPS Rescue (tab Mode), hoặc khi mất sóng nếu chọn ở ô dưới. Sóng trở lại giữa chừng: gạt cần roll hoặc pitch quá 30 % để lấy lại quyền điều khiển. Pin dưới mức tối thiểu thì luôn hạ cánh tại chỗ.":
    "Rescue: the craft climbs to its return height, turns its nose to the take-off point, flies there, comes down gradually, lands and stops the motors. It runs when the GPS Rescue switch is on (Modes tab), or when the link is lost if chosen in the box below. Link back on the way: move the roll or pitch stick by more than 30 % to take control again. A battery below its minimum always lands on the spot.",
  "Sẵn sàng bay về":
    "Ready to fly home",
  "chưa":
    "not yet",
  "Giai đoạn":
    "Phase",
  "Độ cao bay về lần này":
    "Return height this time",
  "\"Sẵn sàng\" chỉ có khi máy đang bay với điểm home (GPS đã fix lúc arm), la bàn và ước lượng độ cao. Đã cài Rescue (công tắc hoặc failsafe) thì firmware không cho arm khi GPS chưa fix hoặc không có la bàn.":
    "\"Ready\" only shows while the craft is flying with a home point (GPS fix when it was armed), a compass and a height estimate. With Rescue set up (switch or failsafe) the firmware refuses to arm without a GPS fix or without a compass.",
  "PMW3901: bốn chân SPI của riêng nó (không chung bus với IMU, và không dùng được cùng lúc với thẻ SD kiểu SPI). Cảm biến MSP qua UART: chỉ cần RX, nối với TX của cảm biến; firmware tự lấy cổng UART còn trống (UART1 nếu không có GPS, UART2 nếu không có receiver nối tiếp, UART0 khi app nối qua USB của chip - RX mặc định của UART0 là GPIO44). Chọn loại cảm biến ở tab Vị trí.":
    "PMW3901: four SPI pins of its own (not the IMU's bus, and it cannot be used together with an SD card in SPI mode). MSP sensor on a UART: only RX is needed, wired to the sensor's TX; the firmware takes whichever UART is free (UART1 without a GPS, UART2 without a serial receiver, UART0 when the app is on the chip's USB - the default RX of UART0 is GPIO44). Choose the sensor type on the Position tab.",
  "ESP32-S3 có ba UART, mỗi cái nối được vào bất kỳ chân nào; chân của từng thiết bị đặt ở các nhóm bên trái. UART0 chỉ dùng được khi app nối qua cổng USB của chip (không qua mạch USB-UART). Tốc độ baud do từng thiết bị tự chọn (GPS tự dò, receiver theo giao thức, optical flow ở tab Vị trí).":
    "The ESP32-S3 has three UARTs, each can be routed to any pin; the pins of each device are set in the groups on the left. UART0 can only be used when the app talks through the chip's own USB port (not through a USB-UART bridge). Each device picks its own baud rate (the GPS is detected, the receiver follows its protocol, the optical flow sensor is set in the Position tab).",
  "Loại cảm biến":
    "Sensor type",
  "PMW3901: nối SPI riêng (4 chân), cần thêm một lidar để biết độ cao. MSP: MicoAir MTF-01 / MTF-02, Matek 3901-L0X nối UART (chỉ cần chân RX), có sẵn lidar.":
    "PMW3901: its own SPI bus (4 pins), and it needs a separate lidar for the height. MSP: MicoAir MTF-01 / MTF-02, Matek 3901-L0X on a UART (only the RX pin), with a lidar built in.",
  "Baud của cảm biến UART":
    "Baud rate of a UART sensor",
  "Hướng lắp cảm biến":
    "Sensor mounting direction",
  "Cảm biến bị xoay bao nhiêu độ theo chiều kim đồng hồ (nhìn từ trên xuống) so với hướng chuẩn của nó.":
    "How far the sensor is turned clockwise (seen from above) from its normal direction.",
  "Ảnh bị lật gương":
    "Mirrored picture",
  "Tần số xung servo":
    "Servo pulse rate",
  "50 cho servo analog, tới 330 cho servo digital. Có hiệu lực sau khi lưu và khởi động lại.":
    "50 for analog servos, up to 330 for digital ones. Takes effect after saving and restarting.",
  "Mỗi servo đi theo một kênh RC: kênh ở 1000 µs cho xung \"nhỏ nhất\", ở 2000 µs cho xung \"lớn nhất\". Có tác dụng ngay; chân servo gán ở tab Chân và cổng (cần lưu và khởi động lại). Nguồn servo lấy từ BEC 5 / 6 V riêng, nối chung GND.":
    "Each servo follows one RC channel: the channel at 1000 µs gives the \"lowest\" pulse, at 2000 µs the \"highest\". Takes effect at once; the servo pins are set on the Pins tab (save and restart). Power the servos from their own 5 / 6 V BEC, grounds joined.",
  "Kênh RC":
    "RC channel",
  "Nhỏ nhất (µs)":
    "Lowest (µs)",
  "Lớn nhất (µs)":
    "Highest (µs)",
  "Đang phát":
    "Sending",
  "Chân xung của servo 1 đến 4; kênh RC chọn ở tab Motor.":
    "Pulse pins of servos 1 to 4; the RC channel is chosen on the Motor tab.",
  "Bật khi hướng lắp đã đúng mà đưa máy sang phải mũi tên lại chỉ sang trái.":
    "Switch on when the mounting direction is right but moving the craft to the right makes the arrow point left.",
  "Hệ số tốc độ":
    "Speed scale",
  "100 = đúng như cảm biến báo. Tốc độ đọc được cao hơn thật thì giảm xuống.":
    "100 = as the sensor reports. Lower it if the speed reads higher than it really is.",
  "Chất lượng bề mặt tối thiểu":
    "Minimum surface quality",
  "Mẫu có chất lượng thấp hơn mức này (0 đến 100) bị bỏ qua.":
    "Samples with a quality below this (0 to 100) are skipped.",
  "Độ cao tối đa dùng optical flow":
    "Maximum height for optical flow",
  "Cao hơn mức này (hoặc ngoài tầm lidar) thì không dùng optical flow.":
    "Above this (or beyond the lidar's range) optical flow is not used.",
  "Chỉnh góc nghiêng theo gia tốc đo được":
    "Correct the tilt from the measured acceleration",
  "Lúc máy tăng tốc hoặc phanh, accelerometer một mình làm góc nghiêng ước lượng lệch vài độ. Bật: dùng vận tốc từ GPS / optical flow để sửa (chỉ khi đang arm).":
    "While the craft speeds up or brakes, the accelerometer alone lets the estimated tilt drift by a few degrees. On: the velocity from GPS / optical flow is used to correct it (only while armed).",
  "Góc nghiêng tối đa của autopilot":
    "Maximum lean of the autopilot",
  "Cho cả Position Hold lẫn GPS Rescue.":
    "For Position Hold and GPS Rescue alike.",
  "Tốc độ nghiêng lại để phanh":
    "Rate of leaning back to brake",
  "PHLD_BRAKE_RATE của ArduCopter.":
    "ArduCopter's PHLD_BRAKE_RATE.",
  "Góc phanh tối đa":
    "Maximum brake angle",
  "PHLD_BRAKE_ANGLE của ArduCopter.":
    "ArduCopter's PHLD_BRAKE_ANGLE.",
  "Vị trí P":
    "Position P",
  "Sai lệch vị trí thành vận tốc (PSC_NE_POS_P của ArduCopter).":
    "Position error to velocity (ArduCopter's PSC_NE_POS_P).",
  "Vận tốc P":
    "Velocity P",
  "Vận tốc I":
    "Velocity I",
  "PSC_NE_VEL_I: phần chống lại gió thổi đều.":
    "PSC_NE_VEL_I: the part that stands against a steady wind.",
  "Vận tốc D":
    "Velocity D",
  "Khi mất sóng hoặc bật công tắc Failsafe":
    "On a lost link or the Failsafe switch",
  "Rescue cần GPS đã fix và la bàn; thiếu một trong hai thì máy hạ cánh tại chỗ.":
    "Rescue needs a GPS fix and a compass; without either the craft lands where it is.",
  "Độ cao bay về":
    "Return height",
  "Leo thêm":
    "Extra climb",
  "Cộng vào điểm cao nhất của chuyến bay, hoặc vào độ cao hiện tại.":
    "Added to the highest point of the flight, or to the present height.",
  "Độ cao cố định":
    "Fixed height",
  "So với điểm cất cánh; dùng khi chọn \"Cố định\".":
    "Above the take-off point; used when \"Fixed\" is chosen.",
  "Tốc độ leo":
    "Climb rate",
  "Tốc độ hạ":
    "Descent rate",
  "Ở gần mặt đất. Trên cao máy hạ nhanh hơn, tối đa gấp ba.":
    "Near the ground. Higher up the craft descends faster, up to three times as fast.",
  "Tốc độ bay về":
    "Speed on the way home",
  "Bắt đầu hạ khi cách home":
    "Start descending at this distance from home",
  "Khoảng cách tối thiểu để bay về":
    "Minimum distance for flying home",
  "Gần home hơn mức này thì máy leo lên rồi hạ tại chỗ.":
    "Closer to home than this the craft climbs and comes down on the spot.",
  "Độ cao bắt đầu hạ chậm":
    "Height where the slow landing begins",
  "Số vệ tinh tối thiểu khi rescue":
    "Minimum satellites during a rescue",
  "Ít hơn mức này suốt 15 giây: bỏ cuộc, hạ cánh tại chỗ.":
    "Fewer than this for 15 seconds: it gives up and lands on the spot.",
  "Cho arm khi GPS chưa fix":
    "Allow arming without a GPS fix",
  "Khi đó không có điểm home: rescue không bay về được, failsafe sẽ hạ cánh tại chỗ.":
    "Then there is no home point: the rescue cannot fly home, the failsafe lands on the spot.",
  "Position Hold đang bật nhưng chưa có gì để giữ vị trí (optical flow + lidar, hoặc GPS đã fix + la bàn đã hiệu chuẩn)":
    "Position Hold is on but there is nothing to hold a position with (optical flow + lidar, or a GPS fix + a calibrated compass)",
  "đã cài GPS Rescue (có công tắc, dù đang tắt, hoặc là thủ tục failsafe) nhưng GPS chưa fix: chưa có điểm home. Không muốn chờ: bật \"Cho arm khi GPS chưa fix\" ở tab Vị trí (như Betaflight)":
    "GPS Rescue is set up (it has a switch, even one that is off, or it is the failsafe procedure) but the GPS has no fix: no home point yet. Not wanting to wait: turn on \"Allow arming without a GPS fix\" in the Position tab (as in Betaflight)",
  "đã cài GPS Rescue nhưng la bàn đang tắt hoặc không có":
    "GPS Rescue is set up but the compass is switched off or missing",
  "Giữ vị trí bằng optical flow hoặc GPS + la bàn; tự bật Angle, nên bật cùng Altitude Hold. Cài đặt ở tab Vị trí.":
    "Holds the position with optical flow or GPS + compass; switches Angle on by itself, best used with Altitude Hold. Settings on the Position tab.",
  "Bay về điểm cất cánh rồi hạ cánh (cần GPS đã fix và la bàn). Tắt công tắc để lấy lại quyền điều khiển. Cài đặt ở tab Vị trí.":
    "Flies back to the take-off point and lands (needs a GPS fix and a compass). Switch off to take control again. Settings on the Position tab.",
  "Vị trí và điều hướng":
    "Position and navigation",
  "Ước lượng vị trí, optical flow, Position Hold, GPS Rescue (10 lần mỗi giây).":
    "Position estimate, optical flow, Position Hold, GPS Rescue (10 times a second).",
  "MSP qua UART (MicoAir MTF, Matek)":
    "MSP on a UART (MicoAir MTF, Matek)",
  "đang dùng":
    "in use",
  "bỏ qua: bề mặt quá mờ hoặc quá tối":
    "skipped: the surface is too blurred or too dark",
  "bỏ qua: quá gần sàn, quá cao, hoặc lidar không có số đo":
    "skipped: too close to the floor, too high, or no lidar reading",
  "bỏ qua: máy nghiêng quá 45°":
    "skipped: the craft is tilted more than 45°",
  "bỏ qua: máy đang quay quá nhanh":
    "skipped: the craft is turning too fast",
  "bỏ qua: chưa đủ dữ liệu gyro":
    "skipped: not enough gyro data yet",
  "phi công điều khiển":
    "pilot in control",
  "đang phanh":
    "braking",
  "phanh xong, chờ trục kia":
    "braked, waiting for the other axis",
  "đang chuyển sang giữ vị trí":
    "changing over to holding",
  "đang trả lại cho phi công":
    "handing back to the pilot",
  "không chạy":
    "not running",
  "đang leo lên độ cao bay về":
    "climbing to the return height",
  "đang quay mũi về home":
    "turning the nose home",
  "đang bay về":
    "flying home",
  "đang hạ dần về home":
    "descending towards home",
  "đang hạ cánh":
    "landing",
  "bỏ cuộc":
    "gave up",
  "không có vị trí từ GPS":
    "no position from the GPS",
  "không có hướng la bàn":
    "no compass heading",
  "không có ước lượng độ cao":
    "no height estimate",
  "mất GPS":
    "GPS lost",
  "quá ít vệ tinh":
    "too few satellites",
  "không tiến được về phía home":
    "making no progress towards home",
  "máy đang bay ra xa home":
    "the craft is flying away from home",
  "Điểm cao nhất của chuyến bay + leo thêm":
    "Highest point of the flight + extra climb",
  "Cố định":
    "Fixed",
  "Độ cao hiện tại + leo thêm":
    "Present height + extra climb",
  "Hạ cánh tại chỗ":
    "Land on the spot",
  "GPS Rescue (bay về rồi hạ cánh)":
    "GPS Rescue (fly home, then land)",
  "Tự nhận":
    "Detect",
  "Lidar của cảm biến optical flow (MSP)":
    "Lidar of the optical flow sensor (MSP)",
  "ADC đo dòng":
    "Current ADC",
  "ADC đo dòng: GPIO{pin} không đo được điện áp trên ESP32-S3 (chỉ GPIO1 đến GPIO20).":
    "Current ADC: GPIO{pin} cannot measure a voltage on an ESP32-S3 (only GPIO1 to GPIO20).",
  "Chân cảm biến dòng chưa đo được điện áp nào. Kiểm tra chân \"ADC đo dòng\" ở tab Chân và cổng (cần khởi động lại sau khi gán chân).":
    "The current sensor pin has not measured any voltage yet. Check the \"Current ADC\" pin in the Pins and ports tab (a restart is needed after setting the pin).",
  "Còn lại":
    "Left",
  "Có số này và có số đo dòng: phần trăm pin còn lại tính theo số mAh đã dùng. 0 = tính theo điện áp.":
    "With this and a current reading, the battery left (%) is worked out from the mAh used. 0 = from the voltage.",
  "Cảm biến dòng (chân ADC)":
    "Current sensor (ADC pin)",
  "Dung lượng pin":
    "Battery capacity",
  "Dòng":
    "Current",
  "Dòng lấy từ cảm biến dòng nối vào một chân ADC, hoặc là tổng dòng các ESC báo về (cần DShot hai chiều và extended telemetry ở tab Motor). Số mAh đã dùng tính từ lúc cấp nguồn.":
    "The current comes from a current sensor on an ADC pin, or is the sum of what the ESCs report (needs bidirectional DShot and extended telemetry in the Motors tab). The mAh used count from power-up.",
  "Dòng đo bằng đồng hồ":
    "Current measured with a meter",
  "ESC (telemetry DShot mở rộng)":
    "ESC (extended DShot telemetry)",
  "Giống ibata_offset của Betaflight: cộng vào dòng tính được (cảm biến có điện áp khác 0 lúc không có dòng thì độ lệch âm).":
    "Like Betaflight's ibata_offset: added to the computed current (a sensor that reads above 0 V with no current needs a negative offset).",
  "Giống ibata_scale của Betaflight: 400 = 40 mV mỗi ampe. Số âm nếu cảm biến lắp ngược chiều. Lấy số của nhà sản xuất board, rồi hiệu chỉnh bên dưới.":
    "Like Betaflight's ibata_scale: 400 = 40 mV per amp. Negative if the sensor is mounted the other way round. Start with the board maker's number, then calibrate below.",
  "Hiệu chỉnh: tháo cánh, cho motor quay đều ở tab Motor, đo dòng bằng ampe kìm hoặc đọc trên nguồn, nhập vào đây. Hai số này có tác dụng ngay, không cần khởi động lại.":
    "Calibration: props off, spin the motors steadily in the Motors tab, measure the current with a clamp meter or read it on the power supply, enter it here. Both numbers take effect at once, no restart needed.",
  "Hệ số cảm biến dòng":
    "Current sensor scale",
  "Hệ số cảm biến mới: {k}. Bấm \"Lưu\" để giữ lại.":
    "New sensor scale: {k}. Press \"Save\" to keep it.",
  "Hệ số tính được ({k}) nằm ngoài khoảng cho phép: kiểm tra độ lệch và dòng nhập vào.":
    "The computed scale ({k}) is out of range: check the offset and the current you entered.",
  "Nguồn đo dòng":
    "Current source",
  "Nguồn đo điện áp: {src}. Nguồn đo dòng: {csrc}.":
    "Voltage from: {src}. Current from: {csrc}.",
  "Nhập dòng đo bằng đồng hồ (0,5 đến 300 A), lúc motor đang quay đều.":
    "Enter the current measured with a meter (0.5 to 300 A), with the motors spinning steadily.",
  "Tính hệ số cảm biến":
    "Compute the sensor scale",
  "Tự chọn (cảm biến nếu có chân, không thì ESC)":
    "Automatic (the sensor if it has a pin, otherwise the ESCs)",
  "cảm biến dòng":
    "current sensor",
  "ví dụ 5.0":
    "e.g. 5.0",
  "Điện áp ở chân cảm biến dòng":
    "Voltage at the current sensor pin",
  "Đo dòng và dung lượng":
    "Current and capacity",
  "Đã dùng":
    "Used",
  "Độ lệch cảm biến dòng":
    "Current sensor offset",
  "0,1 mV/A":
    "0.1 mV/A",
  "Mỗi kênh một dây, kênh 1 trước. Chỉ dùng khi giao thức receiver là PWM. Để trống kênh nào không nối.":
    "One wire per channel, channel 1 first. Only used when the receiver protocol is PWM. Leave the channels that are not wired empty.",
  "Một dây mang mọi kênh. Chỉ dùng khi giao thức receiver (tab Receiver) là PPM; khi đó UART2 để trống cho việc khác.":
    "One wire carries every channel. Only used when the receiver protocol (Receiver tab) is PPM; UART2 is then free for something else.",
  "PPM (một chân)":
    "PPM (one pin)",
  "PPM: một dây tín hiệu vào chân \"PPM\" ở tab Chân và cổng (mức 3,3 V; receiver 5 V cần mạch chia áp). Số kênh (4 đến 12) firmware tự nhận sau khoảng nửa giây. UART2 để trống cho việc khác.":
    "PPM: one signal wire to the \"PPM\" pin in the Pins and ports tab (3.3 V levels; a 5 V receiver needs a divider). The firmware learns the channel count (4 to 12) in about half a second. UART2 stays free for something else.",
  "PWM (mỗi kênh một chân)":
    "PWM (one pin per channel)",
  "PWM: mỗi kênh một dây, chân đặt ở nhóm \"Receiver PWM\" của tab Chân và cổng (mức 3,3 V). Đặt failsafe của receiver là \"không phát xung\": receiver phát xung failsafe thì firmware không biết là đã mất sóng.":
    "PWM: one wire per channel, the pins are set in the \"Receiver PWM\" group of the Pins and ports tab (3.3 V levels). Set the receiver's failsafe to \"no pulses\": a receiver that sends failsafe pulses hides the lost link from the firmware.",
  "Receiver PPM":
    "PPM receiver",
  "Receiver PPM: chưa có chân PPM.":
    "PPM receiver: no PPM pin yet.",
  "Receiver PWM":
    "PWM receiver",
  "Receiver PWM: chưa có chân của kênh 1.":
    "PWM receiver: no pin for channel 1 yet.",
  "Spektrum (satellite DSMX / DSM2) nối vào RX của UART2, 3,3 V. Spektrum gửi kênh theo thứ tự TAER: chọn \"TAER\" ở Thứ tự kênh. Firmware chưa làm được việc bind: bind satellite bằng receiver chính hoặc tay phát trước.":
    "Spektrum (DSMX / DSM2 satellite) on the RX pin of UART2, 3.3 V. Spektrum sends the channels in TAER order: pick \"TAER\" under Channel order. The firmware cannot bind yet: bind the satellite with a main receiver or the transmitter first.",
  "Spektrum 1024 (DSM2 22 ms)":
    "Spektrum 1024 (DSM2 22 ms)",
  "Spektrum 2048 (DSMX, DSM2 11 ms)":
    "Spektrum 2048 (DSMX, DSM2 11 ms)",
  "Gán cổng nối tiếp":
    "Serial port assignment",
  "Tự động: GPS ở UART1, receiver nối tiếp ở UART2, cảm biến optical flow MSP lấy cổng còn trống. Theo bảng: mỗi UART làm việc chọn trong bảng. Có hiệu lực sau khi lưu và khởi động lại.":
    "Automatic: GPS on UART1, serial receiver on UART2, an MSP optical flow sensor on the port left. Table: each UART does what the table says. Takes effect after saving and restarting.",
  "Receiver nối tiếp":
    "Serial receiver",
  "Optical flow MSP":
    "MSP optical flow",
  "Tự động":
    "Automatic",
  "Theo bảng":
    "Table",
  "Cổng nối tiếp: \"{f}\" được chọn cho hơn một UART; firmware dùng UART đầu tiên.":
    "Serial ports: \"{f}\" is chosen for more than one UART; the firmware uses the first one.",
  "Cổng nối tiếp: bảng không có UART nào cho GPS.":
    "Serial ports: the table has no UART for the GPS.",
  "Cổng nối tiếp: bảng không có UART nào cho receiver nối tiếp.":
    "Serial ports: the table has no UART for the serial receiver.",
  "Baud GPS":
    "GPS baud",
  "Firmware tự tìm baud hiện tại của module u-blox (khi bật \"Tự dò baud\") rồi chuyển module sang tốc độ này; nên dùng 115200. Module NMEA phải được đặt sẵn tốc độ này. Đổi cần lưu và khởi động lại.":
    "The firmware finds the u-blox module at whatever rate it runs (with \"Detect baud\" on) and switches it to this one; 115200 is recommended. An NMEA module must already be set to this rate. Takes effect after saving and restarting.",
  "Trả lời nhận được / hỏng mỗi giây: ":
    "Replies received / garbled per second: ",
  "Brushed (motor chổi than, MOSFET)":
    "Brushed (brushed motors, MOSFET)",
  "Khung này có {n} motor: DShot / ProShot trên ESP32-S3 chỉ chạy được 4 (4 bộ phát RMT). Chọn PWM, Oneshot hoặc Multishot.":
    "This frame has {n} motors: DShot / ProShot on the ESP32-S3 can drive only 4 (4 RMT transmitters). Pick PWM, Oneshot or Multishot.",
  "Hexa + (Betaflight)":
    "Hexa + (Betaflight)",
  "Hexa X (thứ tự Betaflight)":
    "Hexa X (Betaflight order)",
  "Y6 (đồng trục)":
    "Y6 (coaxial)",
  "Octo X8 (đồng trục)":
    "Octo X8 (coaxial)",
  "Octo phẳng +":
    "Octo flat +",
  "Octo phẳng X":
    "Octo flat X",
  "Quad X (thứ tự 1234)":
    "Quad X (1234 order)",
  "Dùng độ cao GPS":
    "Use the GPS height",
  "Không dùng: không bao giờ. Chỉ khi không có baro / lidar: máy không lắp barometer (hoặc barometer mất tín hiệu giữa chuyến bay) thì Altitude Hold chạy theo độ cao GPS. Trộn với baro: như trên, và khi có barometer thì độ cao GPS từ từ (khoảng một phút) sửa phần trôi theo thời tiết của barometer, cho các chuyến bay dài.":
    "Off: never. Only without baro / lidar: a craft without a barometer (or whose barometer goes silent in flight) flies Altitude Hold on the GPS height. Blend with baro: as before, and next to a barometer the GPS height slowly (about a minute) takes out the barometer's weather drift, for long flights.",
  "Dùng vận tốc lên / xuống của GPS":
    "Use the GPS vertical velocity",
  "Module u-blox đo được vận tốc đứng khá chính xác (0,1 đến 0,3 m/s), không bị gió cánh quạt làm nhiễu như barometer: tốc độ leo / hạ ước lượng ổn định hơn. Module NMEA không có số này nên không dùng được.":
    "A u-blox module measures the vertical velocity quite well (0.1 to 0.3 m/s) and is not disturbed by prop wash as a barometer is: a steadier estimated climb rate. NMEA modules do not report it.",
  "Chỉ khi không có baro / lidar":
    "Only without baro / lidar",
  "Trộn với baro (sửa trôi baro)":
    "Blend with baro (corrects its drift)",
  "Độ cao từ GPS":
    "Height from the GPS",
  "Độ cao GPS trôi một đến vài mét mỗi phút, barometer thì ổn định hơn trong vài giây nhưng trôi theo thời tiết. Vì vậy mặc định độ cao GPS chỉ dùng khi không có barometer / lidar; còn vận tốc lên / xuống của GPS (u-blox) thì luôn có ích, nhất là trong gió cánh quạt và khi leo nhanh.":
    "The GPS height wanders by one to a few metres a minute; a barometer is steadier over seconds but drifts with the weather. So by default the GPS height is only used without a barometer / lidar, while the GPS vertical velocity (u-blox) always helps, above all in prop wash and fast climbs.",
  "Độ cao GPS":
    "GPS height",
  "Vận tốc đứng GPS":
    "GPS vertical velocity",
  "đang dùng trong ước lượng độ cao":
    "in use for the height estimate",
  "không dùng lúc này":
    "not in use now",
  "Chỉ có điểm home (lúc arm, GPS đã fix) thì mới có độ cao GPS. Lần đầu dùng Altitude Hold chỉ với GPS: bay ở chỗ thoáng, cao trên 5 m, sẵn sàng tắt công tắc.":
    "There is a GPS height only with a home point (GPS fix when arming). The first time you fly Altitude Hold on the GPS alone: open sky, above 5 m, ready to flip the switch off.",
  "Có điện trở kéo lên ngoài":
    "External pull-up resistors",
  "DShot hai chiều: bật khi dây tín hiệu motor đã có điện trở kéo lên 3,3 V trên mạch (thường 1 đến 4,7 kΩ). Khi đó firmware tắt điện trở kéo lên bên trong chân (~45 kΩ). Có hiệu lực sau khi lưu và khởi động lại.":
    "Bidirectional DShot: switch on when the motor signal lines already have pull-up resistors to 3.3 V on the board (usually 1 to 4.7 kOhm). The firmware then leaves the pins' internal pull-up (~45 kOhm) off. Takes effect after saving and restarting.",
  "Khung hỏng mỗi giây: ":
    "Bad frames per second: ",
  "Tràn bộ đệm UART: ":
    "UART buffer overflows: ",
  "Lỗi đường truyền UART: ":
    "UART line errors: ",
  "Tốc độ khung thấp hơn tốc độ gói của receiver (ELRS: 150, 250, 500 Hz…): \"Khung hỏng\" tăng = byte bị mất hoặc sai. \"Tràn bộ đệm\" tăng = firmware đọc UART không kịp. \"Lỗi đường truyền\" tăng = nhiễu trên dây, sai baud hoặc sai đảo mức. Cả ba bằng 0 mà tốc độ vẫn thấp = receiver gửi ít khung hơn (xem tốc độ gói ở tay phát).":
    "Frame rate below the receiver's packet rate (ELRS: 150, 250, 500 Hz…): \"Bad frames\" rising = bytes lost or corrupted. \"Buffer overflows\" rising = the firmware read the UART too late. \"Line errors\" rising = noise on the wire, wrong baud rate or wrong inversion. All three at 0 and the rate still low = the receiver sends fewer frames (check the packet rate on the transmitter).",
  "Dữ liệu từ firmware ngừng {s} giây nhưng firmware KHÔNG khởi động lại (chi tiết: tab Hệ thống, Nhật ký firmware)":
    "No data from the firmware for {s} s, but the firmware did NOT restart (details: System tab, firmware log)",
  "Chưa có số liệu.":
    "No data yet.",
  "Firmware chạy được":
    "Firmware up for",
  "Khung nhận từ app / hỏng":
    "Frames received from the app / broken",
  "Lệnh không được xử lý kịp (BUSY)":
    "Requests not handled in time (BUSY)",
  "Lệnh chờ lâu nhất (1 s qua)":
    "Longest request wait (last second)",
  "Luồng link nghỉ lâu nhất":
    "Longest pause of the link task",
  "Luồng cảm biến nghỉ lâu nhất":
    "Longest pause of the sensor thread",
  "Khung telemetry bỏ qua / trả lời bị mất":
    "Telemetry frames skipped / replies lost",
  "Cổng USB bị kẹt phải kích lại":
    "Stuck USB port restarted",
  "Byte đang chờ gửi":
    "Bytes waiting to be sent",
  "Đường truyền với app":
    "Link to the app",
  "Khi app báo \"firmware không trả lời\": số giây chạy tăng đều nghĩa là firmware không khởi động lại, chỉ là dữ liệu bị nghẽn. Luồng cảm biến nghỉ lâu (hàng chục ms) nghĩa là vòng lặp chiếm hết CPU của core 1: hạ tần số loop. Gửi các số này kèm log khi báo lỗi.":
    "When the app says \"the firmware does not answer\": an uptime that keeps counting means the firmware did not restart, only the data was held up. Long pauses of the sensor thread (tens of ms) mean the loop takes all of core 1: lower the loop rate. Send these numbers with the log when reporting it.",
  "GPS gửi quá ít lời giải mỗi giây để giữ vị trí (xem tần số GPS ở tab Cảm biến)":
    "the GPS sends too few solutions a second to hold a position (see the GPS rate in the Sensors tab)",
  "Tần số lời giải":
    "Solution rate",
  " (chậm: Position Hold bằng GPS cần từ 5 Hz; module NMEA thì tăng baud lên 57600)":
    " (slow: Position Hold on GPS needs 5 Hz or more; for an NMEA module raise the baud rate to 57600)",
  "Tần số GPS tối thiểu để giữ vị trí":
    "Lowest GPS rate for Position Hold",
  "Position Hold chỉ dùng GPS khi module gửi ít nhất từng này lời giải mỗi giây (0 = không kiểm tra). GPS 1 Hz làm vị trí và vận tốc nhảy mỗi giây một lần nên máy lắc liên tục. Không ảnh hưởng tới optical flow.":
    "Position Hold uses the GPS only when the module sends at least this many solutions a second (0 = no check). A 1 Hz GPS makes position and velocity jump once a second and the craft keeps rocking. Optical flow is not affected.",
  "Cần ga trong Altitude Hold":
    "Throttle stick in Altitude Hold",
  "Cần ở \"điểm giữ\" ± vùng chết: giữ độ cao. Đẩy lên tới \"cần tối đa\": leo nhanh nhất; kéo xuống tới \"cần thấp\": hạ nhanh nhất, và trên mặt đất thì đứng yên. Ga treo (mức ga máy treo được) firmware tự đo, không cần nhập. Đường cong ga ở trên không áp dụng cho mode này.":
    "Stick at the \"hold point\" ± the deadband: the height is held. Up to \"full stick\": the fastest climb; down to \"low stick\": the fastest descent, and on the ground it stays there. The hover throttle is measured by the firmware, nothing to enter. The throttle curve above does not apply in this mode.",
  "Điểm giữ độ cao":
    "Hold point",
  "Vị trí cần ga (theo receiver, không qua đường cong ga) giữ nguyên độ cao. Mặc định 50 % = giữa cần.":
    "Throttle stick position (as the receiver sends it, not through the throttle curve) that holds the height. Default 50 % = mid stick.",
  "Vùng chết":
    "Deadband",
  "Nửa bề rộng quanh điểm giữ vẫn còn giữ độ cao, tính theo % hành trình cần.":
    "Half-width around the hold point that still holds the height, in % of the stick travel.",
  "Cần thấp":
    "Low stick",
  "Dưới mức này là \"ga thấp\": hạ với tốc độ tối đa; đang đứng trên đất thì không cất cánh.":
    "Below this is \"throttle low\": the fastest descent; standing on the ground it does not take off.",
  "Cần tối đa":
    "Full stick",
  "Từ vị trí này trở lên: leo với tốc độ tối đa.":
    "From here up: the fastest climb.",
  "Tốc độ leo tối đa":
    "Fastest climb",
  "Tốc độ hạ tối đa":
    "Fastest descent",
  "Cấu hình ESC (BLHeliSuite32, ESC Configurator)":
    "ESC configuration (BLHeliSuite32, ESC Configurator)",
  "EKF (thử nghiệm)":
    "EKF (experimental)",
  "Bộ lọc gộp tư thế, vận tốc, vị trí và bias của IMU vào một: khi la bàn bị nhiễu, vận tốc GPS sửa được hướng mũi mỗi lần máy tăng tốc. Mặc định chỉ chạy song song và ghi vào blackbox (ekf_yaw, ekf_pn…) để so; chọn NAV thì Position Hold và GPS Rescue dùng nó. Chỉ chọn NAV sau khi log cho thấy nó tốt hơn.":
    "One filter for attitude, velocity, position and the IMU's biases: when the compass is disturbed, the GPS velocity corrects the heading every time the craft accelerates. By default it only runs alongside and is written to the blackbox (ekf_yaw, ekf_pn…) for comparison; with NAV, Position Hold and GPS Rescue use it. Choose NAV only after the logs have shown it is better.",
  "chưa chạy / chưa căn":
    "not running / not aligned",
  "có GPS":
    "GPS aiding",
  "không GPS":
    "no GPS",
  "biết hướng":
    "heading known",
  "chưa chắc hướng":
    "heading uncertain",
  "đang dùng để giữ vị trí":
    "used for Position Hold",
  "chỉ ghi log":
    "logged only",
  "Hướng mũi: EKF / AHRS":
    "Heading: EKF / AHRS",
  "Vị trí EKF (bắc / đông)":
    "EKF position (north / east)",
  "Một bước EKF tốn":
    "One EKF step takes",
  "OFF: không chạy. SHADOW: chạy song song, chỉ ghi log (mặc định). NAV: Position Hold và GPS Rescue lấy vị trí, vận tốc, hướng mũi từ EKF khi nó có GPS.":
    "OFF: not run. SHADOW: runs alongside, logged only (default). NAV: Position Hold and GPS Rescue take position, velocity and heading from the EKF while it has GPS.",
  "Chạy song song, chỉ ghi log":
    "Alongside, logged only",
  "Dùng cho Position Hold / Rescue":
    "Used by Position Hold / Rescue",
  "Firmware có ESC passthrough như Betaflight. Cắm pin, tháo cánh, bấm nút dưới (app sẽ tự ngắt kết nối), rồi mở BLHeliSuite32 / ESC Configurator / AM32 Configurator trên cùng cổng và bấm Connect. Motor ngừng phát trong lúc đó; bấm Disconnect trong công cụ thì firmware tự khởi động lại. Không làm được khi đang ARM.":
    "The firmware has ESC passthrough like Betaflight. Plug in the battery, remove the props, press the button below (the app disconnects by itself), then open BLHeliSuite32 / ESC Configurator / AM32 Configurator on the same port and press Connect. The motor output stops meanwhile; pressing Disconnect in the tool restarts the firmware. Not while ARMED.",
  "Nhả cổng cho BLHeliSuite32":
    "Let go of the port for BLHeliSuite32",
  "Xong. App đã nhả cổng: mở BLHeliSuite32 / ESC Configurator trên cổng này rồi bấm Connect.":
    "Done. The app has let go of the port: open BLHeliSuite32 / ESC Configurator on this port and press Connect.",
  "cổng debug (SWDIO)":
    "the debug port (SWDIO)",
  "cổng debug (SWCLK)":
    "the debug port (SWCLK)",
  "{label}: \"{text}\" không phải tên chân STM32 (PA0 đến PK15).":
    "{label}: \"{text}\" is not an STM32 pin name (PA0 to PK15).",
  "{label}: {pin} thường là {what}.":
    "{label}: {pin} is usually {what}.",
  "{pin} đang gán cho cả \"{a}\" và \"{b}\".":
    "{pin} is set for both \"{a}\" and \"{b}\".",
  "Mạch STM32":
    "STM32 board",
  "Chân ghi theo tên: PA0, PB3, PC13 ... Trên STM32 mỗi UART, SPI, I2C và timer chỉ ra được một số chân nhất định; firmware kiểm tra lúc khởi động và ghi lỗi vào báo cáo ở tab Tổng quan nếu chân không đúng chức năng.":
    "Pins are written by name: PA0, PB3, PC13 ... On an STM32 each UART, SPI, I2C and timer reaches only certain pins; the firmware checks this at boot and writes an error into the report on the Overview tab if a pin cannot do its job.",
  "Chân telemetry (T) của các ESC nối chung vào RX của một UART (chân ở tab Chân: Telemetry ESC). Firmware hỏi lần lượt từng ESC bằng bit telemetry của khung DShot; ESC trả lời nhiệt độ, điện áp, dòng, mAh và RPM. Dùng với BLHeli_32, AM32, KISS. Chỉ với DShot. Đủ cho đo pin và nhiệt độ, không đủ nhanh cho bộ lọc RPM (cần DShot hai chiều). Có hiệu lực sau khi lưu và khởi động lại.":
    "The ESCs' telemetry pads (T) joined to the RX of one UART (pin on the Pins tab: ESC telemetry). The firmware asks one ESC at a time with the telemetry bit of the DShot frame; the ESC answers with temperature, voltage, current, mAh and RPM. For BLHeli_32, AM32, KISS. DShot only. Good for the battery and the temperatures, not fast enough for the RPM filter (that needs bidirectional DShot). Takes effect after saving and restarting.",
  "Dây telemetry ESC (KISS)":
    "ESC telemetry wire (KISS)",
  "Dây telemetry ESC, mỗi giây: ":
    "ESC telemetry wire, per second: ",
  "Dây telemetry chung của các ESC (tab Motor: Dây telemetry ESC). Nối vào RX của một UART; firmware lấy UART còn trống (hoặc UART có chức năng Telemetry ESC trong bảng cổng).":
    "The shared telemetry wire of the ESCs (Motors tab: ESC telemetry wire). Goes to the RX of a UART; the firmware takes the UART that is free (or the one with the function ESC telemetry in the port table).",
  "Telemetry ESC":
    "ESC telemetry",
  "Telemetry ESC (dây riêng)":
    "ESC telemetry (own wire)",
  "{ok} trả lời, {crc} hỏng CRC, {lost} ESC không trả lời":
    "{ok} answers, {crc} bad CRC, {lost} ESCs not answering",
  "Với PWM, Oneshot, Multishot và brushed không có \"Motor idle\": ga thấp nhất khi đã arm chính là \"Xung khi arm\" (như min_throttle của Betaflight).":
    "PWM, Oneshot, Multishot and brushed have no \"Motor idle\": the lowest armed throttle is the \"Armed pulse\" itself (like Betaflight's min_throttle).",
  "Tần số gyro":
    "Gyro rate",
  "Số mẫu gyro đọc mỗi giây; bộ lọc gyro chạy ở tần số này. Firmware tự hạ xuống nếu chip gyro không cho đủ mẫu (IMU qua I2C tối đa 1000 Hz).":
    "Gyro samples read per second; the gyro filters run at this rate. The firmware lowers it when the gyro chip cannot deliver that many samples (an IMU on I2C: 1000 Hz at most).",
  "Chia tần số PID":
    "PID rate divider",
  "Phần điều khiển (tư thế, mode, PID, mixer, gửi lệnh motor) chạy mỗi n mẫu gyro: tần số PID = tần số gyro / n, như pid_process_denom của Betaflight. Ví dụ gyro 8000 Hz, chia 2 = PID 4000 Hz: bộ lọc vẫn thấy mọi mẫu mà CPU nhẹ hơn nhiều. Firmware không cho tần số PID xuống dưới 500 Hz.":
    "The control part (attitude, modes, PID, mixer, motor commands) runs on every n-th gyro sample: PID rate = gyro rate / n, like Betaflight's pid_process_denom. Example: gyro 8000 Hz, divider 2 = PID 4000 Hz: the filters still see every sample while the CPU has much less to do. The firmware never lets the PID rate fall below 500 Hz.",
  "Đang chạy":
    "Running at",
  "Vòng PID thực tế":
    "Actual PID loop",
  "{hz} Hz (đặt {target} Hz = gyro / {n})":
    "{hz} Hz (set {target} Hz = gyro / {n})",
  "IMU thứ hai":
    "Second IMU",
  "Chip IMU thứ hai trên cùng bus SPI với IMU đầu (chung SCK, MISO, MOSI), chỉ cần chân CS riêng. Chọn cách dùng ở tab Cảm biến (Dùng gyro nào). Để trống nếu mạch chỉ có một IMU.":
    "A second IMU chip on the first IMU's SPI bus (shared SCK, MISO, MOSI): only its own CS pin is needed. Choose how it is used on the Sensors tab (Gyro to use). Leave empty if the board has one IMU.",
  "IMU thứ hai dùng chung bus SPI của IMU đầu: IMU đầu cũng phải nối SPI (có chân CS).":
    "The second IMU shares the first IMU's SPI bus: the first IMU must be on SPI as well (with a CS pin).",
  "Dùng gyro nào":
    "Gyro to use",
  "Khi mạch có hai IMU (tab Chân: IMU thứ hai), giống gyro_to_use của Betaflight. IMU 1: bay bằng IMU đầu, IMU 2 dự phòng. IMU 2: bay bằng gyro của IMU 2. Cả hai: lấy trung bình hai gyro (nhiễu thấp hơn). Gia tốc kế luôn lấy từ IMU 1 (chip được hiệu chuẩn 6 mặt). Một chip ngừng trả lời giữa chuyến bay thì chip còn lại bay tiếp.":
    "When the board has two IMUs (Pins tab: Second IMU), like Betaflight's gyro_to_use. IMU 1: fly on the first IMU, IMU 2 as a stand-by. IMU 2: fly on the gyro of IMU 2. Both: the mean of the two gyros (less noise). The accelerometer always comes from IMU 1 (the chip of the 6-face calibration). If one chip stops answering in flight, the other one flies on.",
  "Hướng chip IMU thứ hai":
    "Second IMU chip orientation",
  "Như \"Hướng chip IMU trên mạch\" nhưng cho chip thứ hai (Betaflight gyro_2_sensor_align). Phần chỉnh tinh của mạch áp dụng cho cả hai chip.":
    "Like \"IMU chip orientation on the board\" but for the second chip (Betaflight gyro_2_sensor_align). The fine board trims apply to both chips.",
  "IMU 1 (IMU 2 dự phòng)":
    "IMU 1 (IMU 2 stand-by)",
  "Cả hai (trung bình)":
    "Both (mean)",
  "không tìm thấy lúc khởi động":
    "not found at boot",
  "đã ngừng trả lời":
    "stopped answering",
  "{state}, vòng bay dùng {src}; gyro IMU 2 {g2} (IMU 1 {g1}) °/s":
    "{state}, the loop flies on {src}; IMU 2 gyro {g2} (IMU 1 {g1}) °/s",
  "WiFi của máy bay (điểm phát riêng, 192.168.4.1:5760)":
    "The craft's WiFi (its own access point, 192.168.4.1:5760)",
  "Địa chỉ cho app":
    "Address for the app",
  "Sóng":
    "Signal",
  "Thiết bị đang nối vào":
    "Devices connected",
  "App đang nối qua":
    "The app is connected via",
  "cổng serial (USB / UART)":
    "the serial port (USB / UART)",
  "Có hiệu lực sau khi lưu và khởi động lại. Điểm phát riêng (AP): mạch là 192.168.4.1, tên mặc định Windburst-XXXX, mật khẩu mặc định windburst - hãy đổi mật khẩu, ai vào được mạng này cũng điều khiển được máy (kể cả thử motor). Ở app chọn cổng \"WiFi của máy bay\", hoặc nhập socket://<địa chỉ>:5760 khi máy nối vào mạng nhà bạn (STA).":
    "Takes effect after saving and rebooting. Own access point (AP): the board is 192.168.4.1, default name Windburst-XXXX, default password windburst - change the password: whoever joins this network controls the craft (motor test included). In the app pick the port \"The craft's WiFi\", or type socket://<address>:5760 when the craft joins your home network (STA).",
  "Trên ESP32-S3 các chân ADC2 (GPIO11 đến GPIO20) không đọc được khi WiFi bật: chân đo pin và đo dòng phải nằm ở GPIO1 đến GPIO10.":
    "On the ESP32-S3 the ADC2 pins (GPIO11 to GPIO20) cannot be read while WiFi is on: the battery and current sensor pins must be on GPIO1 to GPIO10.",
  "Tắt; Điểm phát riêng (AP): máy bay tự phát mạng WiFi, máy tính nối vào đó; Nối vào mạng có sẵn (STA): máy bay vào mạng WiFi nhà bạn (tên và mật khẩu bên dưới).":
    "Off; Own access point (AP): the craft makes a WiFi network of its own, the computer joins it; Join a network (STA): the craft joins your WiFi network (name and password below).",
  "Tên mạng (SSID)":
    "Network name (SSID)",
  "AP: để trống thì là Windburst-XXXX (bốn ký tự cuối của địa chỉ MAC). STA: tên mạng cần vào.":
    "AP: empty = Windburst-XXXX (the last four characters of the MAC address). STA: the network to join.",
  "Mật khẩu":
    "Password",
  "Từ 8 đến 63 ký tự (WPA2). AP: để trống thì là windburst. STA: để trống cho mạng không mật khẩu.":
    "8 to 63 characters (WPA2). AP: empty = windburst. STA: empty for an open network.",
  "Kênh (AP)":
    "Channel (AP)",
  "Cổng TCP":
    "TCP port",
  "Cổng app nối vào. Mặc định 5760, giống trình mô phỏng.":
    "The port the app connects to. 5760 by default, like the simulator.",
  "Điểm phát riêng (AP)":
    "Own access point (AP)",
  "Nối vào mạng có sẵn (STA)":
    "Join a network (STA)",
  "đang phát mạng":
    "access point running",
  "đang nối vào mạng…":
    "joining the network…",
  "đã nối vào mạng":
    "connected to the network",
  "không khởi động được (xem nhật ký firmware)":
    "could not start (see the firmware log)",
  "(chưa biết tên)":
    "(name not known yet)",
  "Cấp node id cho module":
    "Hand out node ids",
  "Cần một bộ thu phát CAN 3,3 V (SN65HVD230, TJA1051T/3...) và hai chân ở tab Chân (CAN bus). Module trên bus: GPS provider, loại la bàn, loại khí áp kế, nguồn đo điện áp / dòng chọn DRONECAN; ESC trên bus: giao thức ESC DRONECAN (tab Motor). Có hiệu lực sau khi lưu và khởi động lại.":
    "Needs a 3.3 V CAN transceiver (SN65HVD230, TJA1051T/3...) and two pins in the Pins tab (CAN bus). Modules on the bus: set GPS provider, compass type, barometer type, voltage / current source to DRONECAN; ESCs on the bus: ESC protocol DRONECAN (Motor tab). Takes effect after saving and restarting.",
  "Flight controller ArduPilot dùng 10.":
    "ArduPilot flight controllers use 10.",
  "Module DroneCAN dùng 1 Mbit/s.":
    "DroneCAN modules use 1 Mbit/s.",
  "Node id của mạch":
    "This board's node id",
  "Nối với bộ thu phát CAN 3,3 V: TX vào chân D (TXD), RX từ chân R (RXD). Bật ở tab Cảm biến (CAN bus).":
    "To a 3.3 V CAN transceiver: TX to its D (TXD) pin, RX from its R (RXD) pin. Switched on at the Sensors tab (CAN bus).",
  "Phần lớn module GPS / la bàn DroneCAN tự xin node id lúc khởi động (dynamic node id allocation): mạch cấp cho chúng. Tắt nếu bus đã có máy chủ cấp id khác.":
    "Most DroneCAN GPS / compass modules ask for a node id at power-up (dynamic node id allocation): the board hands them out. Switch it off if another allocation server is on the bus.",
  "Tắt, hoặc DroneCAN: mạch là một node DroneCAN; GPS và la bàn đặt là DRONECAN lấy số liệu từ module trên bus.":
    "Off, or DroneCAN: the board is a DroneCAN node; a GPS and a compass set to DRONECAN take their data from the modules on the bus.",
  "Tốc độ bus":
    "Bus speed",
  "bus-off (đang tự khôi phục)":
    "bus-off (recovering by itself)",
  "la bàn":
    "compass",
  "mất kết nối":
    "offline",
  "{state}, node {id}; {rx} khung/s nhận, {tx} gửi, {drop} bỏ; lỗi TX {tec} / RX {rec}, bus-off {off} lần; đã cấp {alloc} node id":
    "{state}, node {id}; {rx} frames/s in, {tx} out, {drop} dropped; TX errors {tec} / RX {rec}, bus-off {off} times; {alloc} node ids handed out",
  "chạy {s} s":
    "up {s} s",
  "đang dùng: {what}":
    "in use: {what}",
  "Ba trục cùng lúc, mỗi trục một màu. Chạy từng motor ở tab này và xem gyro thô: motor hoặc cánh nào làm gyro rung mạnh hơn hẳn các motor khác là motor cần kiểm tra (cong trục, lỏng ốc, cánh mẻ).":
    "All three axes at once, one colour each. Run one motor at a time on this tab and watch the raw gyro: a motor or prop that shakes the gyro much more than the others is the one to check (bent shaft, loose screw, chipped prop).",
  "Tín hiệu":
    "Signal",
  "thô {raw} °/s RMS, sau lọc {filt} °/s RMS":
    "raw {raw} °/s RMS, filtered {filt} °/s RMS",
  "Motor dừng giữa chừng":
    "A motor that stops",
  "Lần gần nhất: ":
    "Last time: ",
  "motor {n} đã dừng (máy đã disarm)":
    "motor {n} stopped (the craft was disarmed)",
  "chưa có lần nào":
    "never so far",
  "Cần tốc độ của từng motor: DShot hai chiều hoặc dây telemetry ESC. Motor không có tốc độ thì không bị xét. Có hiệu lực sau khi lưu và khởi động lại.":
    "Needs each motor's speed: bidirectional DShot or the ESC telemetry wire. A motor without a speed is not judged. Takes effect after saving and rebooting.",
  "một motor vừa dừng giữa lúc bay (chạm cánh, kẹt motor, mất đồng bộ) nên máy đã disarm: kiểm tra cánh và motor (gạt ARM về OFF để xoá)":
    "a motor stopped in flight (prop strike, blocked motor, desync) and the craft was disarmed: check the props and motors (ARM switch OFF to clear)",
  "Disarm khi một motor dừng":
    "Disarm when a motor stops",
  "Cánh chạm vật, motor bị kẹt hay mất đồng bộ: motor đang được ra lệnh quay mà tốc độ tụt xuống thì tắt hết motor, trước khi các motor còn lại lật máy xuống đất hoặc motor kẹt bị cháy. Mặc định tắt.":
    "A prop strike, a blocked motor or a desync: a motor that is told to turn but slows down switches every motor off, before the others flip the craft into the ground or the blocked motor burns. Off by default.",
  "Coi là dừng dưới":
    "Stalled below",
  "Hoặc tụt còn dưới":
    "Or dropped below",
  "phần trăm tốc độ cao nhất của motor đó kể từ lần lệnh của nó giảm gần nhất":
    "percent of that motor's highest speed since its command last went down",
  "Chỉ xét khi lệnh từ":
    "Judged from a command of",
  "Dưới mức này (ví dụ đang idle trên mặt đất) thì motor không bị xét.":
    "Below it (idling on the ground, say) a motor is not judged.",
  "Kéo dài":
    "Lasting",
  "0 = không có dải LED.":
    "0 = no LED strip.",
  "Chân dữ liệu (DIN) của dải LED WS2812 / SK6812 (tab Pin và còi: Đèn LED).":
    "The data input (DIN) of a WS2812 / SK6812 LED strip (Battery and beeper tab: LED strip).",
  "Dải LED WS2812 / SK6812 nối vào chân LED (tab Chân, nhóm Khác). Xanh lá: sẵn sàng arm; vàng nhấp nháy: còn điều gì chặn arm; khi bay: màu theo mode (Acro trắng, Angle xanh dương, Stabilize xanh lơ, Altitude Hold tím, Position Hold xanh ngọc, GPS Rescue cam nhấp nháy); đỏ nháy nhanh: failsafe; đỏ nháy đôi: pin cạn. Gửi qua bus SPI thứ hai, nên không dùng được cùng thẻ SD kiểu SPI hoặc PMW3901. Có hiệu lực sau khi lưu và khởi động lại.":
    "A WS2812 / SK6812 strip on the LED pin (Pins tab, Other). Green: ready to arm; amber blinking: something blocks arming; flying: the mode's color (Acro white, Angle blue, Stabilize cyan, Altitude Hold purple, Position Hold teal, GPS Rescue orange flashing); fast red flash: failsafe; red double blink: battery empty. Sent through the second SPI bus, so not together with an SD card on SPI or a PMW3901. Takes effect after saving and rebooting.",
  "Kiểu hiển thị":
    "Layout",
  "Mọi LED cùng màu":
    "Every LED the same color",
  "Một LED trắng ở 100 % ăn khoảng 60 mA.":
    "One white LED at 100 % draws about 60 mA.",
  "Số LED":
    "Number of LEDs",
  "Đèn LED (WS2812)":
    "LED strip (WS2812)",
  "Đèn hàng không (trái đỏ, phải xanh, đuôi chớp trắng)":
    "Navigation lights (left red, right green, white tail strobe)",
  "Độ sáng":
    "Brightness",
  "ESC DroneCAN: lệnh ga đi trên bus CAN (esc.RawCommand, 400 lần/giây), ESC index = slot motor (thứ tự motor vẫn áp dụng). Không cần chân motor; CAN bus phải bật (tab Cảm biến). Vòng quay, điện áp, dòng, nhiệt độ lấy từ esc.Status của ESC. Không có lệnh DShot (beacon, đổi chiều): chiều quay đặt trong cấu hình của ESC.":
    "DroneCAN ESCs: the throttle goes over the CAN bus (esc.RawCommand, 400 times a second), ESC index = motor slot (the motor order still applies). No motor pins; the CAN bus must be on (Sensors tab). Speed, voltage, current and temperature come from the ESCs' esc.Status. No DShot commands (beacon, direction): set the spin direction in the ESC's own configuration.",
  "ESC: đã gửi {n} lệnh, bỏ {drop}; nghe thấy ESC {list}":
    "ESC: {n} commands sent, {drop} dropped; heard ESC {list}",
  "(chưa có)":
    "(none yet)",
  "khí áp kế":
    "barometer",
  "pin":
    "battery",
  "Loại khí áp kế":
    "Barometer type",
  "Tự dò: BMP280 / BMP388 trên bus I2C. DRONECAN: áp suất từ module DroneCAN trên bus CAN (cần bật CAN bus).":
    "Auto: a BMP280 / BMP388 on the I2C bus. DRONECAN: the pressure of a DroneCAN module on the CAN bus (the CAN bus must be on).",
  "DroneCAN (ESC trên bus CAN)":
    "DroneCAN (ESCs on the CAN bus)",
  "Module nguồn DroneCAN":
    "DroneCAN power module",
  "module nguồn DroneCAN":
    "DroneCAN power module",
  "Tự dò (BMP280 / BMP388 trên I2C)":
    "Auto-detect (BMP280 / BMP388 on I2C)",
};

// Strings that read the same in both languages (units, protocol names, terms pilots use in English).
export const SAME = [
  "Hz", "°", "S", "mV", "ms", "s", "Airmode", "Roll", "Pitch", "Throttle", "Yaw", "%", "Throttle mid", "Throttle expo", "Throttle limit",
  "Angle strength", "TPA rate", "TPA breakpoint", "I-term relax", "Feedforward max rate limit", "µs", "Extended DShot telemetry", "Motor idle",
  "× 100 rpm", "Dynamic idle P", "Dynamic idle I", "Dynamic idle D", "Barometer", "Lidar / sonar", "GPS", "Baud", "bps", "SBAS", "SBAS integrity",
  "Quad X", "Quad +", "Hexa X", "0°", "90°", "180°", "270°", "PWM", "Oneshot125", "Oneshot42", "Multishot", "DShot150", "DShot300", "DShot600",
  "DShot1200", "Telemetry", "ARM", "Angle", "Stabilize", "Altitude Hold", "Position Hold", "Failsafe", "Beeper", "Blackbox", "Disarm",
  "File", "Firmware", "Gyro", "Motor {n}", "Accelerometer", "Setpoint", "MHz", "500 = Q 5.", "Angle feedforward", "Angle feedforward smoothing",
  "Anti Gravity", "Anti Gravity P gain", "Anti Gravity cutoff", "Anti Gravity gain", "Biquad", "Cutoff", "D-term", "D-term filter multiplier",
  "D-term lowpass 1", "D-term lowpass 1: {n} Hz", "D-term lowpass 2", "Damping", "Drift - Wobble", "Dynamic Damping", "Dynamic Damping (D Max)",
  "Dynamic Damping advance", "Dynamic Damping gain", "Dynamic notch", "Dynamic notch pitch", "Dynamic notch roll", "Dynamic notch yaw", "Feedforward",
  "Feedforward averaging", "Feedforward boost", "Feedforward jitter reduction", "Feedforward smoothness", "Feedforward transition",
  "Feedforward yaw hold gain", "Feedforward yaw hold time", "Gyro filter multiplier", "Gyro lowpass 1", "Gyro lowpass 1: {n} Hz", "Gyro lowpass 2",
  "I-term relax, cutoff", "Master Multiplier", "PID sum (roll, pitch, yaw)", "PT1", "PT2", "PT3", "Pitch Damping", "Pitch Tracking", "Q",
  "RC smoothing", "RPM filter", "RPM notch", "RPM notch: {list} Hz", "Roll, pitch", "Roll, pitch, yaw", "Stabilize P", "Stick Response", "TPA",
  "Throttle boost", "Throttle boost cutoff", "Tracking", "Yaw lowpass", "dynamic notch {axis}", "°/s²",
  "optical flow", "m", "°/s", "1/s", "PSC_NE_VEL_P.", "PSC_NE_VEL_D.", "m/s", "GPS Rescue", "PMW3901 (SPI)", "VL53L1X", "VL53L0X",
  "mA", "mAh",
  "ProShot1000",
  "Hexa H", "Y4", "V-tail quad", "A-tail quad",
  "EKF",
  "gyro {gyro} Hz, PID {pid} Hz",
  "IMU 2",
  "WiFi",
  "1 Mbit/s", "125 kbit/s", "250 kbit/s", "500 kbit/s", "CAN bus", "CAN bus (DroneCAN)", "DroneCAN", "Node {id}",
  "rpm", "ms",
];
