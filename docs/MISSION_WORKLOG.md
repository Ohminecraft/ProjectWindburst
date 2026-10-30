# Nhật ký làm "Nhiệm vụ bay" (lúc bạn vắng)

Bạn dặn (2026-10-29): cứ làm, chỗ nào vướng thì ghi vào đây rồi làm tiếp. Mỗi mục: chuyện gì, tôi đã quyết thế nào, bạn cần xem lại không.

Thiết kế: `docs/superpowers/specs/2026-10-29-mission-design.md`.

## Quyết định tôi tự đưa ra (bạn xem lại khi về)

1. **Nạp kế hoạch qua WBP ở phần 1, không phải MAVLink.** Trong chat tôi nói phần 1 sẽ nhận kế hoạch theo giao thức nhiệm vụ MAVLink. Khi đọc mã thì thấy firmware chưa có đường *nhận* MAVLink nào (UART MAVLink và UDP WiFi chỉ gửi đi). Vì vậy kế hoạch được lưu theo đúng ngữ nghĩa lệnh MAVLink/ArduPilot (cùng số `MAV_CMD`), nhưng nạp qua WBP: USB, WiFi của ESP32 (`socket://<ip>:5760`, đã có sẵn) hoặc bản mô phỏng. Giao thức `MISSION_COUNT/ITEM_INT/ACK` để sang phần 3 (ELRS), lúc đó mới có đường nhận.
2. **Tối đa 32 lệnh mỗi kế hoạch** (giới hạn 512 byte của một section WBP). Một điểm "hạ cánh · chờ · cất cánh" tốn 3 lệnh, một bước servo tốn 2 lệnh.
3. **Arm lại giữa chuyến bỏ qua 3 điều kiện**: "cần ga chưa ở thấp", "công tắc ARM đang bật", "mất sóng RC". Mọi điều kiện khác vẫn chặn. Lý do: bạn đã chốt "mất sóng thì làm tiếp nhiệm vụ", nên lúc drone đang chờ ở chỗ giao đồ mà mất sóng thì nó vẫn phải tự cất cánh được. Bạn gạt công tắc ARM về OFF trong lúc nó chờ là nó không tự cất cánh.
4. **Kế hoạch sửa trong lúc đang bay chỉ có hiệu lực ở lần bắt đầu sau.** ArduPilot cho sửa giữa chừng; tôi bỏ qua cho đơn giản và an toàn.

5. **Phần 2 (tab bản đồ) cũng đã làm**, theo bố cục C bạn chọn. Bản đồ dùng Leaflet đóng gói sẵn trong app. Ảnh nền đi qua server `wbclient` để cache lên đĩa (`~/Windburst/tiles`), vì luật dùng ô bản đồ của OpenStreetMap yêu cầu có cache và có tên chương trình.
6. **Điểm giao hàng cất cánh lại lên đúng độ cao của chính điểm đó** (ô "Cất cánh lại lên"). Độ cao bay giữa các điểm lấy theo từng điểm.
7. **Tốc độ bay mặc định = tốc độ của rescue** (`rescue_ground_speed`, mặc định 7,5 m/s), không phải 5 m/s như tôi ghi trong thiết kế: như vậy chỉ có một chỗ chỉnh. Muốn khác thì đặt "Tốc độ" trong tab Nhiệm vụ (thành lệnh DO_CHANGE_SPEED).

8. **Mutation test:** 93 mutant cho phần nhiệm vụ, tất cả bị diệt. Tôi bỏ 4 mutant vì chúng không đổi được hành vi:
   - `want_arm` đã bị chặn ở bước trên khi công tắc ARM tắt;
   - `holds_home` không cần thêm `running()`;
   - nhánh Position Hold nằm sau nhánh nhiệm vụ nên không bao giờ giành được;
   - `reset_position` khi nhiệm vụ arm lại vẫn tự hồi phục, vì home GPS đã được giữ.

   Hai mutant đầu tôi xử lý bằng cách đơn giản hoá code cho khớp.
9. **"Vị trí của tôi" chỉ để Windows hỏi quyền** (bạn yêu cầu). WebView2 bình thường hiện hộp hỏi riêng trước, rồi Windows hỏi lần nữa. Trang là của app, nên `desktop.py` (`allow_location`) cho đúng trang này xin vị trí mà không qua hộp của WebView2. Quyền thật vẫn do Windows giữ (Cài đặt > Quyền riêng tư và bảo mật > Vị trí). Trang khác hoặc quyền khác (camera...) vẫn đi qua hộp của WebView2. Nếu pywebview đổi cách làm bên trong thì hook tự bỏ qua, chỉ hiện thêm hộp của WebView2.

## Vướng mắc / cần bạn quyết

1. **Chưa thử trên mạch.** Mọi thứ mới kiểm trên PC: unit test, mô phỏng vòng kín, firmware thật trên mạch ảo và test trình duyệt.
2. **F405: RAM 88,6 %** (trước là 86,5 %). Kế hoạch được giữ 3 bản: trong cài đặt (450 byte), trong core và trong bộ chạy (mỗi bản khoảng 0,9 KB). Nếu F405 thiếu RAM thì có thể bỏ bớt một bản.
3. **Cỡ giao diện app (đã kiểm, không lỗi):** đo trong Edge/WebView2 ở 80 %, 100 % và 125 %. Bấm thêm điểm, kéo điểm và kéo bản đồ đều đúng tới từng pixel. Đã thêm test `test_mission_tab_themes_and_zoom` để giữ như vậy.
4. **Chữ ở thanh tìm kiếm (đã sửa, bạn báo):** nút bản đồ đang chọn dùng màu tự chế, nên ở theme Swiss thành chữ đen trên nền đen, ở Glass chữ gần như trong suốt; khi rê chuột lên ở theme mặc định thì thành chữ trắng trên nền xanh nhạt. Cách sửa:
   - Nút đang chọn dùng kiểu `primary` mà theme nào cũng tô sẵn, có thêm `aria-pressed`.
   - Ô tìm kiếm là input text như các ô khác.
   - Các ô nổi có lớp nền đặc dưới màu panel, nên đọc được trên ảnh vệ tinh ở Glass.

   Test kiểm độ tương phản của nút đang chọn ở mọi theme (tối thiểu 3:1), và chiều cao ô tìm kiếm phải bằng các ô khác.
5. **Các test trước bị lỗi vì thời gian (đã sửa):**
   - `test_e2e.py::test_streams_show_the_craft`: chỉ xét lưới 20 ms từ khung thứ 5 trở đi. Mấy khung đầu có thể còn theo pha cũ trước khi lệnh đăng ký lại có hiệu lực, và số khung đó phụ thuộc tốc độ máy.
   - `test_blackbox.py::test_flight_logged_to_flash_downloaded_and_decoded`: so thời lượng log với thời gian arm đo bằng đồng hồ của mô phỏng (lệch dưới 0,3 s), thay cho giới hạn cứng 10 s. Test này giờ chặt hơn trước.
   - `test_gui_viewer.py::test_pid_tuning_panel`: thời gian chờ phân tích log tăng từ 30 s lên 90 s.
   - Các lỗi termios trên Windows và các test trình duyệt đã lỗi từ trước thì vẫn như cũ.
6. **Thanh cuộn của cả trang loé lên khi đổi tab (đã sửa, bạn báo):** tab mới trượt vào từ 12 px bên dưới, nên trong 180 ms trang cao hơn cửa sổ 12 px và trình duyệt hiện thanh cuộn. Test cũ không thấy lỗi này vì hiệu ứng tự tắt khi trình duyệt do test điều khiển (`navigator.webdriver`). Test mới bật hiệu ứng lên, chạy mọi theme, và đã bắt được lỗi (tràn 12 px) trước khi sửa. Cách sửa: `#app { overflow: clip }`.
7. **Tab Nhiệm vụ cao hơn khung 147 px (đã sửa):** tab có thanh cuộn riêng, và bấm lên bản đồ làm trang nhảy 76 px (Leaflet focus vào bản đồ, trình duyệt cuộn tới nó). Giờ bản đồ lấp vừa phần còn lại của khung tab.
8. **`test_pid_tuning_panel` thỉnh thoảng hết giờ (đã sửa):** lỗi không phải do máy chậm (tôi từng nâng thời gian chờ lên 90 s, việc đó vô ích). Lúc trang vừa mở, tab đang xem bị dựng lại tới 2 lần: một lần khi hash đổi trong lúc chờ danh sách cổng, một lần ở sự kiện "chưa kết nối" đầu tiên. Tab Blackbox vì vậy mở lại log, và kết quả phân tích PID rơi vào khung đã bị bỏ. `main.js` giờ chỉ vẽ tab đầu tiên một lần, và không dựng lại tab dùng được khi không kết nối nếu trạng thái kết nối không đổi. Kèm theo: bấm thanh tổng quan của đồ thị giờ vẽ khung nhìn mới ngay (`logplot.js`), trước đó phải chờ dữ liệu về; `test_overview_click_moves_playhead_and_window` từng lỗi khoảng 1/17 lần vì chỗ này. Kết quả: chạy cả file viewer 25 lần liền đều qua.

