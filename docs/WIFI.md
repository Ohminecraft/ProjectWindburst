# Nối app qua WiFi (0.14.0)

Chỉ có trên ESP32 có radio, ví dụ ESP32-S3. STM32 không có. Firmware báo tính năng `WIFI` trong HELLO; app chỉ hiện khối WiFi ở tab Hệ thống khi có tính năng này.

| Setting (section `wifi`, cần khởi động lại) | Ý nghĩa |
|---|---|
| `mode` | `OFF` (mặc định) / `AP`: máy bay tự phát mạng, mạch là 192.168.4.1 / `STA`: vào mạng WiFi có sẵn |
| `ssid` | AP: để trống thì tên là `Windburst-XXXX` (đuôi địa chỉ MAC). STA: tên mạng cần vào |
| `pass` | 8 – 63 ký tự (WPA2). AP: để trống thì là `windburst`. STA: để trống cho mạng không mật khẩu |
| `channel` | kênh WiFi của AP (1 – 13) |
| `port` | cổng TCP, mặc định 5760 (giống trình mô phỏng) |

`ssid` và `pass` là chuỗi UTF-8 đệm 0 (mảng byte trong schema, cờ `text`; `pass` thêm cờ `secret`). App hiện chúng thành ô chữ, `pass` là ô mật khẩu.

## Cách chạy
- `app/wifi_link.h` (`WifiLink`): bật radio bằng `WiFi.h`. Phần TCP dùng socket lwIP không chặn (non-blocking), nên mạng chậm hay mất cũng không làm luồng link phải chờ.
  - Mỗi lúc chỉ một client. Kết nối mới thay kết nối cũ, để một client "chết" mà không đóng socket không chặn link mãi.
  - Ở STA, khi mất mạng radio tự nối lại.
- `app/link_mux.h` (`LinkMux`): vẫn một `wbp::Server` duy nhất. Khung trả lời và stream đi về **dây mà app vừa nói**: serial (USB / UART0) hoặc WiFi. Client WiFi rời đi thì quay về serial.
- ESC passthrough (BLHeliSuite32 …) chỉ qua cổng serial.
- Báo cáo khởi động: `wifi: AP 'Windburst-1A2B' channel 6, app at socket://192.168.4.1:5760`, hoặc `wifi: joining '<mạng>' …`. Ở STA, địa chỉ có được sau khi vào mạng và hiện ở tab Hệ thống.
- Stream `link` thêm `wifi` (OFF / AP / CONNECTING / CONNECTED / FAILED), `wifi_ip`, `wifi_rssi`, `wifi_clients`, `via` (SERIAL / WIFI).
- Blackbox: section `wifi` **không** được ghi vào header log, để mật khẩu không theo file log đi ra ngoài. Bản sao lưu settings (.json) thì vẫn có mật khẩu.

## Dùng
1. Nối USB, tab Hệ thống → WiFi: chọn AP, đặt mật khẩu, bấm Lưu (firmware khởi động lại).
2. Máy tính vào mạng `Windburst-XXXX`. Ở app chọn cổng "WiFi của máy bay (192.168.4.1:5760)" rồi bấm Kết nối.
3. STA: máy tính và máy bay cùng một mạng. Chọn "Nhập cổng khác…" và gõ `socket://<địa chỉ ở tab Hệ thống>:5760`.

## Lưu ý
- **An toàn**: ai vào được mạng này đều điều khiển được máy, kể cả thử motor. Hãy đổi mật khẩu mặc định.
- **ADC2**: trên ESP32-S3, khi WiFi bật thì các chân ADC2 (GPIO11 – GPIO20) không đọc được. Chân đo pin / đo dòng phải ở GPIO1 – GPIO10; báo cáo khởi động sẽ cảnh báo nếu đặt sai.
- Radio dùng khoảng 50 kB RAM và chạy ở core 0 (cùng core với link / blackbox). Vòng bay ở core 1. **Chưa đo trên mạch** WiFi có làm vòng bay giật hay không: xem "Luồng link nghỉ lâu nhất" và loop Hz khi đang nối qua WiFi.
- App không dò 192.168.4.1. Mục WiFi luôn có trong danh sách cổng nhưng không bao giờ là mặc định, vì một lần dò sẽ cướp kết nối của app đang dùng WiFi.

## Kiểm thử
- `test_app` nhóm "link mux": khung đi theo dây app vừa nói, client mất thì về serial, WiFi chậm thì phần thừa chờ trong hàng đợi. Kiểm cả các trường WiFi của stream `link`.
- `test_proto`: mặc định và giới hạn của section `wifi`. `test_fw`: header blackbox không có `set wifi`.
- Trình duyệt: `test_wifi_settings_text_fields` (ô chữ UTF-8, mật khẩu cắt ở 63 byte, ô mật khẩu ẩn). `test_gui_api`: mục cổng WiFi.
- Mutant: `python3 tools/mutate.py wifi`.
- **Phần radio / socket không chạy được trên PC**: mới chỉ biên dịch cho ESP32-S3.
