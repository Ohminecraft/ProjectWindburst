# Blackbox — `src/blackbox`, `app/wbclient/wblog.py`, tab Blackbox

Ghi lại chuyến bay để xem bằng đồ thị: gyro, setpoint, các thành phần PID, cần điều khiển, lệnh motor, góc, thời gian vòng lặp (nhanh), và trạng thái, mode, pin, độ cao, GPS, la bàn, vị trí và điều hướng (10 lần/giây).

## 1. Hai đường đi của log

| | Flash trên mạch | Thẻ SD | Gửi thẳng về app ("ghi trực tiếp qua cáp") |
|---|---|---|---|
| Bật bằng | `blackbox.device = FLASH` + `blackbox.trigger` (`ARMED` / `SWITCH` = mode Blackbox / `ALWAYS`) | `blackbox.device = SDCARD` + `trigger` | nút "Bắt đầu ghi" trong tab, hoặc `python -m wbclient blackbox record` |
| Lấy về | tải qua đường truyền app (`BLACKBOX_READ`) | rút thẻ cắm vào máy tính (file `/wblog/wb_00001.wblog`…), hoặc tải qua app như flash | app ghi thành file ngay khi nhận |
| Giới hạn | dung lượng phân vùng (N16R8: 14 MB); mỗi lần ghi flash làm đứng cả hai nhân (mục 4) | dung lượng thẻ; thẻ chậm thì mất bản ghi (được đếm), **vòng bay không bị ảnh hưởng** | tốc độ đường truyền (230400 baud ≈ 250 bản ghi/giây với các nhóm mặc định) |

Nếu mạch có thẻ SD thì nên dùng thẻ: nó là đường duy nhất không làm vòng bay đứng (mục 4b).

Quy tắc giữa hai đường (yêu cầu gốc: "khi kết nối với app, blackbox to Storage tự tắt và truyền qua serial"; đã sửa theo góp ý **không tự chuyển giữa lúc bay**):
- App xin luồng log khi máy **ở mặt đất**: phiên ghi flash đang chạy (nếu có) được đóng lại, luồng bắt đầu.
- App xin luồng log khi máy **đang arm và đang ghi flash**: bị từ chối (`BUSY`), log của chuyến bay không bị cắt.
- Trong lúc app nhận luồng thì không mở phiên flash mới. Luồng kết thúc khi app bảo dừng hoặc app im lặng 3 s; sau đó `trigger` lại quyết định.
- Chỉ kết nối app thôi thì **không** làm tắt việc ghi flash; phải bấm "Bắt đầu ghi".
- **Tốc độ của luồng trực tiếp tự giới hạn theo đường truyền**: không quá nửa số byte mỗi giây mà cổng chở được (UART: baud / 10; USB CDC kiểu TinyUSB: 60 kB/s; USB Serial/JTAG: không giới hạn). Yêu cầu 1000 bản ghi/giây qua cầu 230400 baud thì firmware tự hạ xuống mức vừa đường truyền, thay vì bỏ khung lung tung. Stream `blackbox.rate_hz` là tốc độ thật.
- **Tải log** (`BLACKBOX_READ`): đang xoá, hoặc đang arm và đang ghi → `BUSY`. Đã disarm mà phiên còn mở (trigger ALWAYS, hoặc vừa disarm) → firmware đóng phiên đó, trả `BUSY` cho lần hỏi này, client thử lại trong 3 giây. Trong 3 giây sau một lần đọc, firmware không mở phiên mới (arm thì bỏ giới hạn này ngay). GUI: bấm tải lúc đang ghi trực tiếp thì phiên ghi trực tiếp được dừng trước.

Không có: mạch hiện thành ổ USB (mass storage), nén dữ liệu.

## 2. Định dạng `.wblog`

File = chuỗi **khung WBP** (cùng khung, cùng CRC với đường truyền, xem `docs/PROTOCOL.md`), nên file trong flash, luồng gửi về app và file trên máy tính là một định dạng:

| Khung | Nội dung |
|---|---|
| `BB_HEADER` 0x60 | văn bản, cắt thành nhiều khung (seq 0, 1, 2 …). Seq 0 mở một phiên mới và bắt đầu bằng `WBLOG 1` |
| `BB_DATA` 0x61 | các bản ghi nối nhau: `'F'` nhanh `[t_us u32][các trường]`, `'S'` chậm (10 Hz), `'E'` sự kiện `[t_us][mã][n][dữ liệu]` |
| `BB_BLOB` 0x62 | chỉ trên đường truyền: một mẩu của bộ nhớ log khi tải về (`[offset u32][dữ liệu]`) |

Header **tự mô tả**: mỗi trường ghi `tên:kiểu:scale:đơn vị` (ví dụ `gyro[0]:i16:0.1:deg/s`), kèm tên sự kiện, phiên bản firmware, schema hash, `loop_hz` (tần số PID, từ 0.12), `gyro_hz`, `rate_hz`, số motor, và **toàn bộ settings của chuyến bay** (`set <section> <hex>`). Trình đọc chỉ dựa vào header, nên log của bản firmware khác vẫn đọc được; settings kèm theo chỉ giải mã được khi schema khớp.

Sự kiện: `ARM`, `DISARM`, `FAILSAFE`, `MODES`, `DROPPED` (firmware bỏ bản ghi vì hàng đợi đầy), `BATTERY`, `END` (lý do kết thúc), `AH_STATE`. Một phiên không có `END` = mất nguồn lúc đang ghi; dữ liệu trước đó vẫn dùng được. Khung hỏng hay thiếu chỉ mất các bản ghi bên trong nó (đếm trong `gaps`).

## 3. Firmware

```
src/blackbox/bb_format.h      bảng trường (kFast, kSlow), nhóm, Plan (chọn trường), đóng gói bản ghi          [portable]
src/blackbox/bb_ring.h        hàng đợi SPSC không khóa: vòng bay ghi, task link đọc                             [portable]
src/blackbox/bb_recorder.h    bb::Recorder: phía vòng bay (log_fast mỗi N chu kỳ, log_slow 10 Hz, sự kiện)      [portable]
src/blackbox/bb_flash.h       bb::FlashLog: các phiên trên một vùng NOR thô (BlockStore)                        [portable]
src/blackbox/bb_storage.h     bb::Storage: giao diện chung của nơi lưu (flash hoặc thẻ)                         [portable]
src/blackbox/bb_card.h        bb::CardLog: mỗi phiên một file trên thẻ SD; hàng đợi byte, task ghi riêng         [portable]
src/blackbox/bb_card_esp.h    SdFileStore (Arduino SD / SD_MMC), SdCardTask (task "wb_sd"), RtosLock
src/blackbox/bb_service.h     bb::Service: phía task link - trigger, header, flash / thẻ / luồng, tải về, xoá   [portable]
src/blackbox/bb_header.h      phần header do firmware thêm (phiên bản, settings)
src/blackbox/bb_store_esp.h   EspPartitionStore: phân vùng "wblog", không có thì phân vùng "spiffs" có sẵn
partitions/wblog_{4,8,16}MB.csv   bảng phân vùng có phân vùng log lớn (platformio.ini: board_build.partitions)
```
- **Vòng bay chỉ ghi vào RAM** (hàng đợi 16 kB). Đầy thì bỏ bản ghi và ghi sự kiện `DROPPED`; vòng bay không bao giờ chờ.
- **Flash là vùng NOR thô, không phải hệ thống file.** Lý do: file system sẽ tự xoá sector giữa lúc bay (đứng 45 ms trở lên). Ở đây việc xoá chỉ xảy ra khi người dùng bấm "Xoá bộ nhớ log"; lúc bay chỉ có ghi trang.
  - Mỗi phiên bắt đầu ở đầu một sector 4 kB; tìm các phiên lúc khởi động bằng cách đọc dấu đầu sector.
  - Vùng trống được **kiểm tra là toàn 0xFF** trước khi dùng. Phân vùng chứa dữ liệu khác (SPIFFS cũ…) → trạng thái `NEEDS_ERASE`, firmware không ghi đè.
  - Tối đa **một trang 256 byte mỗi mili-giây**; trang chưa đầy được ghi sau 0,5 s (giới hạn dữ liệu mất khi mất nguồn).
  - Xoá từ cuối về đầu, từng khối, nên xoá dở (mất nguồn) không để lại phiên "ma"; trong lúc xoá không arm được (blocker `CALIBRATING`), xoá xong còi kêu.
- **Tốc độ ghi** `blackbox.rate_hz` được làm tròn về ước số của tần số vòng lặp. `blackbox.fields` chọn nhóm trường; mặc định 74 byte mỗi bản ghi nhanh.
- **Đơn vị của các cột PID** (`pid_p / pid_i / pid_d / pid_f`): đơn vị PID sum của Betaflight, 1000 = toàn dải motor (tổng bị giới hạn ±500, yaw ±400), bước 0,1. Tới bản 0.6.1 các cột này bị kẹp ở ±3,28 vì sai tỉ lệ khi ghi; log cũ vẫn đọc được nhưng số PID trong đó không dùng được (`docs/HARDWARE_FEEDBACK_2.md` mục 2).
- **Nhóm "Thời gian vòng lặp"**: `loop_us` (thời gian chạy của chu kỳ được ghi), `gap_us` (khoảng cách dài nhất giữa hai chu kỳ kể từ bản ghi trước), và từ 0.6.2 năm cột một byte, bước 4 µs, mỗi cột là giá trị **lớn nhất** kể từ bản ghi trước: `t_lock` (chờ khoá), `t_imu` (đọc IMU), `t_core` (tính toán, chu kỳ thường), `t_est` (tính toán, chu kỳ có chạy bộ ước lượng; 0 = không có), `t_motor` (gửi motor).
- **Nhóm "Vị trí và điều hướng"** (từ 0.7.0, `NAV`; 10 lần/giây như các nhóm chậm khác): `pos_n / pos_e / vel_n / vel_e` (chuyển từ nhóm GPS sang đây: có cả khi chỉ có optical flow), `est_valid` (bit của schema: độ cao / vị trí / hướng từ la bàn / vận tốc / flow / góc nghiêng được chỉnh), `nav_flags` (Position Hold, Rescue, autopilot đang cho góc nghiêng), `flow_quality`, `flow_reject` (vì sao mẫu flow mới nhất không được dùng), `flow_fwd / flow_right` (tốc độ flow đo được, theo trục máy), `nav_roll / nav_pitch` (góc nghiêng autopilot yêu cầu), `nav_target_n / nav_target_e` (điểm Position Hold đang giữ), `rescue_phase`. Settings lưu từ bản trước 0.7.0 không có bit của nhóm này: bật nó ở tab Blackbox (mặc định của 0.7.0 đã có).
- Mọi thay đổi ở section `blackbox` là **live**; phiên đang ghi giữ cấu hình lúc nó bắt đầu.
- Stream `blackbox` (id 11): trạng thái (`OFF / NO_STORAGE / NEEDS_ERASE / IDLE / LOGGING / STREAMING / ERASING / FULL`), thiết bị đang chọn, dung lượng và đã dùng (byte, và kB cho thẻ lớn hơn 4 GB), số phiên, tốc độ thật, byte mỗi bản ghi, số bản ghi bị bỏ, % xoá, mức đầy cao nhất của bộ đệm thẻ.
- `device = NONE`: không ghi gì, nhưng các phiên đang nằm trong flash vẫn liệt kê / tải / xoá được. Khi logging tắt, firmware không đọc kiểm tra vùng flash nữa.

### Thẻ SD (`bb_card.h`)

```
vòng bay (nhân 1) --bản ghi--> hàng đợi 16 kB --mỗi ms--> task link: đóng khung --> ByteFifo (512 kB trong PSRAM)
                                                                                         |
task "wb_sd" (nhân 0, ưu tiên thấp):  CardLog::work()  <---- khối 4 kB, sync mỗi 250 ms --+
```
- **Vì sao cần task riêng**: thẻ trả lời một lần ghi trong 1 đến 5 ms, nhưng thỉnh thoảng 50 đến 300 ms (thẻ tự dọn dẹp bên trong). Task link (telemetry, app) và vòng bay không được chờ chuyện đó. Trong lúc ghi, task link chỉ chép byte vào hàng đợi; mọi lệnh `write()` / `sync()` nằm ở task `wb_sd`. Những việc ở mặt đất (mở / đóng file, liệt kê, đọc lại, xoá) dùng chung một khóa nên hai task không bao giờ đụng hệ thống file cùng lúc.
- **Không làm đứng vòng bay**: ghi thẻ không tắt cache flash, và chạy ở nhân khác. Đây là khác biệt chính so với ghi flash trên mạch.
- **File**: thư mục `/wblog`, tên `wb_00001.wblog`, `wb_00002.wblog`… (số tăng dần, tiếp theo số lớn nhất có trên thẻ). Nội dung giống hệt log trong flash hay log app ghi qua cáp: mở thẳng bằng tab Blackbox ("Thêm file .wblog…") hoặc `python -m wbclient blackbox info`.
- **Mất nguồn giữa chừng** (pin văng khi rơi): mất nhiều nhất phần ghi sau lần sync gần nhất (250 ms) cộng phần còn trong bộ đệm. File vẫn đọc được tới đó. Flash trên mạch mất ít hơn (vài chục ms): nếu cần từng mili-giây cuối trước cú rơi thì cân nhắc flash.
- **Bộ đệm**: 512 kB PSRAM khi module có PSRAM (N16R8), không thì 32 kB RAM trong. Ở 2000 bản ghi/giây (≈ 150 kB/giây) 512 kB chịu được thẻ đứng hơn 3 giây; 32 kB chỉ khoảng 0,2 giây. Đầy bộ đệm thì bản ghi bị bỏ và **được đếm** (sự kiện `DROPPED` trong log, `dropped` và `buffer_pct = 100` trong stream) — vòng bay vẫn không bị gì.
- **Thẻ chưa cắm / rút ra / cắm lại**: không có thẻ → `NO_STORAGE`, cứ 3 s tìm lại, không cần khởi động lại. Rút thẻ lúc đang ghi → phiên kết thúc ở đó, file đọc được tới chỗ đã ghi; cắm lại thì ghi tiếp sang file mới. Lúc rảnh firmware kiểm tra thẻ còn đó mỗi 3 s.
- **Chọn thẻ trong app không cần khởi động lại** (thẻ được mount ngay bởi task ghi). Đổi **chân** thì cần lưu và khởi động lại như mọi chân khác. Đang chọn thiết bị khác thì firmware không đụng tới thẻ và các chân của nó.
- **Xoá**: chỉ xoá các file `wb_*.wblog` trong `/wblog`; file khác trên thẻ không bị đụng tới. Nhanh (xoá file, không ghi đè).
- **Tải qua app**: các file được xếp nối nhau thành một dải địa chỉ, nên app tải bằng đúng các lệnh `BLACKBOX_SESSION` / `BLACKBOX_READ` như với flash. Liệt kê 128 file mới nhất (và không quá 4 GB tổng cộng); file cũ hơn vẫn nằm trên thẻ.
- **Nối dây** (`blackbox.sd_mode`, chân `pins.sd_*`):
  - `SPI`: CS, SCK, MOSI, MISO. Dùng bus SPI thứ hai của ESP32-S3 (bus thứ nhất là của IMU): thẻ không bao giờ chung dây với gyro. Barometer qua SPI cũng dùng bus thứ hai — không dùng được cùng lúc với thẻ SPI (barometer qua I2C thì không sao).
  - `SDMMC_1BIT`: CLK, CMD, D0 (ba chân thẻ giống SCK, DI, DO). Nhanh hơn, không chờ bận. Cần trở kéo lên 10 kΩ ở CMD và D0, và chân DAT3 / CS của thẻ giữ ở mức cao.
  - `blackbox.sd_mhz` (mặc định 20): dây dài mà thẻ không nhận thì hạ xuống 4 đến 10.
- Thẻ phải định dạng **FAT32** (FAT16 cũng được). Bản FatFs trong Arduino-ESP32 **không có exFAT**: thẻ 64 GB trở lên xuất xưởng là exFAT, phải định dạng lại thành FAT32 trên máy tính. Firmware không tự định dạng thẻ.

### Xem danh sách không cắt bản ghi (0.8.8)
Lỗi người dùng gặp: ghi blackbox bằng công tắc AUX, mở tab Blackbox của app thì phiên đang ghi bị đóng, 3 s sau một phiên mới tự bắt đầu, lần xem danh sách kế tiếp lại đóng nó... Từ 0.8.8:
- `BLACKBOX_SESSION` (xem danh sách) **không đóng** phiên đang ghi nữa; phiên đó có trong danh sách với kích thước tới lúc đó.
- Chỉ khi **tải** (`BLACKBOX_READ`) phiên mới bị đóng (trên mặt đất; đang bay thì BUSY như cũ). App gọi `blackbox_claim()` trước khi tải để danh sách có kích thước cuối cùng.
- Phiên của **công tắc** mà app đã đóng để tải: không tự bắt đầu lại khi công tắc vẫn bật; gạt công tắc tắt rồi bật để ghi phiên mới. (Trigger ALWAYS vẫn tự ghi tiếp 3 s sau lần đọc cuối; luồng trực tiếp về app kết thúc thì công tắc vẫn ghi tiếp như cũ.)
Kiểm chứng: `test_blackbox` (TRIG_SWITCH: xem danh sách 5 lần trong 4 s → một phiên, vẫn ghi; tải → đóng, 5 s sau vẫn không có phiên mới; tắt / bật → phiên mới), pytest `test_download_with_trigger_always`.

## 4. Điều phải đo trên mạch thật: flash làm đứng vòng bay

Trên ESP32, **mọi thao tác flash tắt cache và làm đứng cả hai nhân**, kể cả nhân đang chạy vòng bay. Thiết kế trên giới hạn mỗi lần đứng ở một trang (theo datasheet khoảng 0,7 ms), không bao giờ xoá lúc bay. Trong mô phỏng: ghi trang 700 µs, xoá sector 45 ms, xoá khối 64 kB 150 ms — **đây là số giả định, chưa đo**.

Hệ quả nếu số thật lớn hơn: vòng 4 kHz mất vài chu kỳ mỗi lần ghi trang (ở 250 bản ghi/giây là khoảng 70 lần ghi mỗi giây). Firmware tự ghi lại điều này để bạn đo:
- trường `gap_us` trong log (nhóm "Thời gian vòng lặp") = khoảng cách dài nhất giữa hai chu kỳ kể từ bản ghi trước;
- `status.gap_max_us` trong telemetry.

Việc đầu tiên nên làm trên mạch: bật ghi flash, arm **không cánh**, xem `gap_us`. Nếu lớn hơn khoảng 1,5 ms thì nên dùng "ghi trực tiếp qua cáp" hoặc giảm `rate_hz`, và báo lại để đổi thiết kế (ví dụ ghi ra flash SPI ngoài hoặc thẻ SD trên bus riêng).

## 4b. Thẻ SD: điều phải đo trên mạch thật

Trong mô phỏng, một lần ghi thẻ mất 3 ms, cứ 25 lần thì một lần mất 180 ms, sync 6 ms — **số giả định**. Vòng bay trong mô phỏng không mất chu kỳ nào (4000 Hz, `gap_us` luôn 250), vì việc ghi nằm ở task khác; trên mạch thật điều này phụ thuộc vào việc thư viện SD của Arduino-ESP32 không làm gì chặn nhân 1. Cách đo: chọn thẻ, bật nhóm "Thời gian vòng lặp", arm **không cánh** một phút, xem `gap_us` trong log và `buffer_pct` / `dropped` ở tab Blackbox.

## 5. Dung lượng

Bảng phân vùng mặc định của Arduino-ESP32 (4 MB) chỉ có phân vùng `spiffs` khoảng 1,5 MB ≈ 75 giây ở 250 bản ghi/giây với các nhóm mặc định (khoảng 20 kB/giây). Muốn nhiều hơn: dùng một bảng trong `partitions/` (bỏ ghi chú dòng `board_build.partitions` trong `platformio.ini`) cho đúng dung lượng flash của mạch, giảm `rate_hz`, hoặc bỏ bớt nhóm. Tab Blackbox tính sẵn "byte mỗi bản ghi, khoảng … mỗi giây, chỗ trống đủ cho khoảng …".

## 6. App

**Tab Blackbox** (`web/js/tabs/blackbox.js`, đồ thị `web/js/gfx/logplot.js`, phía server `wbclient/logview.py` + các đường `/api/blackbox/*`, `/api/log/*` trong `gui.py`):
- *Ghi gì, khi nào*: nơi ghi, điều kiện, tốc độ, các nhóm trường, ước tính dung lượng.
- *Bộ nhớ log trên mạch*: trạng thái, phần đã dùng, danh sách phiên, tải từng phiên hoặc tất cả (có tiến độ), xoá (hỏi lại trước). Khóa khi đang arm.
- *Ghi trực tiếp qua cáp*: đồ thị chạy theo dữ liệu mới ("Bám theo dữ liệu mới"); dừng thì thành một file trong thư viện.
  - Từ 0.31 cửa sổ đồ thị trôi theo đồng hồ, khoảng 30 hình/giây (`web/js/gfx/follow.js`), không nhảy theo nhịp dữ liệu về. Trước đó, dữ liệu qua cầu UART / driver USB của máy tính về thành từng cục nên đồ thị nhảy khoảng 1 lần/giây.
  - Cửa sổ kết thúc hơi lùi sau bản ghi mới nhất (`lag`): lag tự chỉnh theo khoảng cách giữa hai đợt dữ liệu (×1,25), nhỏ nhất 0,1 s, lớn nhất 2 s. Nhờ vậy mép phải không bao giờ chạy vào chỗ chưa có dữ liệu.
  - Tốc độ trôi chỉ được chỉnh ±40 % để bám mục tiêu: không bao giờ nhảy, không bao giờ lùi.
  - App lấy dữ liệu mới 5 lần/giây.
  - Firmware: test FIL `test_fw` nhóm `[blackbox] live stream over a 230400 baud UART` mô phỏng UART 230400 baud sau hàng đợi 8 kB. Khung đi đều, cách nhau tối đa khoảng 20–30 ms, kể cả khi có thêm telemetry 50 Hz; không mất khung nào; bản ghi mới nhất cũ chưa tới 80 ms khi đến app. Như vậy chỗ dồn cục không nằm ở firmware.
- *Log trên máy tính*: thư mục `~/Windburst/logs` (đổi bằng `gui --logs THƯ_MỤC` hoặc biến môi trường `WBCLIENT_LOG_DIR`); xem, lưu file, xoá, thêm file `.wblog` có sẵn. **Xem log không cần nối mạch.**
- *Đồ thị*: nhiều khung xếp chồng chung trục thời gian, mỗi khung một đơn vị (gyro và setpoint từng trục, PID, motor, cần điều khiển, góc, gyro trước / sau lọc, vòng lặp, độ cao, pin, GPS, vị trí (vận tốc ước lượng và của flow, vị trí và điểm đang giữ, góc nghiêng autopilot, chất lượng bề mặt), receiver, la bàn — chỉ hiện nhóm mà log có trường). Nhóm PID vẽ theo đơn vị PID sum (khung nhỏ nhất ±20); nhóm "Vòng lặp" có hai khung: thời gian chạy / khoảng cách giữa hai vòng, và từng khâu của một chu kỳ (log từ 0.6.2). Kéo chuột để phóng to, bấm đúp để xem cả log, Ctrl + lăn chuột, Shift + kéo để dịch; con trỏ đọc giá trị ở mọi khung; sự kiện là vạch đứt có tên; xuất CSV; tải settings của chuyến bay dưới dạng file sao lưu (khôi phục được ở tab Hệ thống).
- Log dài hơn số điểm ảnh: server gửi **giá trị thấp nhất và cao nhất của từng cột điểm ảnh**, nên một gai rộng một mẫu vẫn hiện ở mọi mức thu phóng; phóng to đủ thì gửi chính các mẫu. Chỗ log thiếu dữ liệu thì đường vẽ đứt, không nối qua. "Thiếu" = khoảng cách giữa hai mẫu lớn hơn 2,5 lần **nhịp thật của log** (trung vị khoảng cách các mẫu), không phải nhịp cấu hình: vòng lặp chạy chậm hơn tần số đặt thì log thưa đều, không phải thủng (bản 0.5 vẽ log như vậy thành toàn chấm rời).

**Phát lại kiểu Blackbox Explorer (0.31)** (`web/js/gfx/playback.js`, `logplot.js`):
- Hàng nút phát:
  - về đầu log;
  - lùi / tới một bản ghi;
  - phát / dừng;
  - tốc độ 5 % … 400 %.
- Thanh tổng quan cả chuyến bay: vẽ cần ga (hoặc ga ra motor, hoặc gyro roll), có vạch sự kiện, đánh dấu vùng đang xem và vạch phát.
  - Bấm hoặc kéo trên thanh để nhảy tới chỗ đó.
- Vạch phát (đỏ) đi xuyên mọi khung. Khi phát, cửa sổ luôn giữ vạch phát ở giữa, và chú thích hiện giá trị tại vạch phát.
- Dữ liệu được tải trước một khung phía trước, nửa khung phía sau.
- Phím:
  - Space: phát / dừng;
  - ← →: từng bản ghi (Shift: mười bản ghi);
  - + −: đổi tốc độ;
  - Home / End: về đầu / cuối log.
- Bấm vào đồ thị (không kéo) để đặt vạch phát tại đó.
- Khung trạng thái máy tại vạch phát (lấy dữ liệu riêng, không phụ thuộc nhóm đồ thị đang chọn):
  - hai cần mode 2;
  - cột motor;
  - roll / pitch / yaw;
  - ga ra motor.
- Test: `tests/test_gui_playback.py` (node) và `tests/test_gui_viewer.py` (trình duyệt thật, không cần mạch).

**Phân tích PID từ log (0.31)** (`wbclient/tune.py`, `/api/log/tune`, nút "Phân tích PID từ log này", dòng lệnh `python -m wbclient blackbox tune flight.wblog`):
- **Đáp ứng bước** từng trục, như PIDtoolbox:
  - lấy các cửa sổ 2 s có đánh cần trên 20 °/s, nhiều nhất 12 cửa sổ mạnh nhất;
  - nhân cửa sổ Hann rồi deconvolution Wiener: h = IFFT(G·S̄ / (|S|² + λ)), λ = 1e-5·max|S|²; đáp ứng bước là tổng cộng dồn của h trong 500 ms;
  - bỏ các cửa sổ cho giá trị cuối ngoài khoảng 0,5–1,5, rồi lấy trung bình.
  - Trên hệ bậc hai biết trước (test), sai số dưới 0,04, có nhiễu gyro 4 °/s thì dưới 0,1.
- **Các số đo:**
  - đỉnh (1,0 = không vọt lố);
  - thời gian tới nửa bước;
  - giá trị cuối (200–500 ms);
  - số lần dao động qua 1,0;
  - nhiễu gyro trên khoảng 80 Hz;
  - phần của D trong tổng PID.
- **Gợi ý** (mỗi lần tối đa ±25 %):
  - vọt lố trên 10 % → thêm D và D max; nếu D đã nhiễu thì bớt P;
  - chậm (tới nửa bước sau hơn 18 ms) mà không vọt lố → thêm P / FF;
  - dao động từ 3 lần trở lên → bớt P;
  - giá trị cuối dưới 0,95 → thêm I;
  - nhiễu → không tăng D.
- Nút "Đưa gợi ý vào cấu hình (chưa lưu)": chỉ hiện khi đang nối mạch và cấu hình của chuyến bay không dùng slider.
- Cần log có nhóm "Gyro và setpoint", tốt nhất ghi từ 1000 bản ghi/giây.
- Không cần numpy; nếu có numpy thì dùng cho nhanh hơn.

**Thư viện**
```python
from wbclient import wblog
for s in wblog.load("flight.wblog"):                 # một file có thể có nhiều phiên
    print(s.meta["firmware"], s.duration, len(s.fast["t"]))
    roll_rate = s.fast["gyro[0]"]                    # array('d'), đơn vị kỹ thuật; s.fast["t"] = giây
    print(s.settings(schema)["pid"]["p"])            # settings của chuyến bay đó
    for t, name, data in s.events: ...
```
`WbClient`: `blackbox_status()`, `blackbox_sessions()`, `blackbox_download(index)`, `blackbox_erase()`, `blackbox_record(seconds, rate_hz)`.

**Dòng lệnh**
```
python -m wbclient --port COM5 blackbox status | list | download [N] [-o THƯ_MỤC] | erase
python -m wbclient --port COM5 blackbox record bench.wblog --seconds 10 --hz 200
python -m wbclient blackbox info flight.wblog          # không cần mạch
python -m wbclient blackbox csv flight.wblog [--slow] [-o flight.csv]
```

## 7. Đã kiểm chứng (PC)

- `test_blackbox` 165 kiểm tra: đóng gói, hàng đợi, các phiên trên flash mô phỏng đúng ngữ nghĩa NOR (chỉ ghi được 1 → 0), mất nguồn giữa chừng, dữ liệu lạ trong phân vùng, xoá dở, vùng đầy, giới hạn một trang mỗi ms, tải về, luồng. Trình đọc trong test (`test/wblog_reader.h`) viết độc lập với firmware.
- `test_fw` (firmware-in-the-loop, 294 kiểm tra tất cả): bay trong mô phỏng với mô hình đứng vòng lặp khi ghi flash, tải log về, giải mã, **so với những gì máy ảo thật sự làm** (góc, setpoint, motor, độ cao, pin, GPS, sự kiện); quy tắc flash / luồng; xoá chặn arm.
- 69 lỗi cố ý ở firmware đều bị bắt (`tools/mutate.py`).
- Python: `test_blackbox.py` (trình đọc với log viết tay từng byte, bay → flash → tải → giải mã, luồng trực tiếp, dữ liệu lạ, CLI), `test_logview.py` (đọc log đang lớn dần, thư viện file, khung nhìn rút gọn không làm mất cực trị), `test_gui_api.py` (tải có tiến độ, ghi trực tiếp, từ chối khi đang bay), `test_gui_browser.py` (trang thật: cấu hình, bay, tải, **giá trị đọc ở con trỏ so với log, vị trí nét vẽ so với trục thời gian**, thu phóng, sự kiện, xuất, xoá, ghi trực tiếp, xem log không nối mạch, trang tiếng Anh).
- Thẻ SD: `test_blackbox` (tên file, hàng đợi byte, khối 4 kB và sync 250 ms, bảng phiên, mất nguồn = chỉ còn phần đã sync, thẻ bị rút / cắm lại, thẻ chưa định dạng / chống ghi / đầy, hơn 128 file, hơn 4 GB, xoá chỉ file của mình, chọn / bỏ chọn / đổi kiểu nối) với thẻ mô phỏng có độ trễ và mất nguồn; `test_fw`: **bay ghi ra thẻ có độ trễ 180 ms, vòng bay không mất chu kỳ nào, file trên thẻ = bản tải qua app = những gì máy ảo làm**, không có thẻ lúc khởi động, rút thẻ lúc đang bay, bộ đệm 32 kB với thẻ rất chậm (bản ghi mất được đếm đủ). Python: `test_blackbox.py` (chọn thẻ, bay, tải, rút / cắm, xoá, khởi động lại), trang thật trong trình duyệt.
- Firmware biên dịch cho ESP32-S3 N16R8 (`partitions/wblog_16MB.csv`) và N8; thư viện SD / SD_MMC / FATFS của Arduino-ESP32 có trong file ELF.

## 8. Chưa kiểm chứng (cần mạch thật)

- **Thời gian đứng thật khi ghi flash** (mục 4) — quan trọng nhất.
- `esp_partition_find/read/write/erase_range`: mới biên dịch, chưa chạy. Phân vùng `spiffs` mặc định có được tìm thấy không; thời gian xoá thật.
- Tốc độ tải log qua USB CDC gốc và qua cầu UART; ở 230400 baud tải 1,5 MB mất khoảng 1 đến 2 phút (ước tính).
- Luồng trực tiếp ở tốc độ cao trên đường truyền thật (trong mô phỏng đường truyền không giới hạn tốc độ).
- Trang đồ thị mới chạy trong Chromium không đầu; chưa thử với log dài hàng chục phút (server rút gọn dữ liệu nên dự kiến ổn, chưa đo).
- **Thẻ SD thật**: toàn bộ `bb_card_esp.h` (Arduino `SD` / `SD_MMC`, `File`, `flush()`) mới biên dịch, chưa chạy. Chưa biết: thẻ có mount được ở 20 MHz với dây của bạn không; `usedBytes()` trên thẻ 32 GB mất bao lâu lúc khởi động; độ trễ ghi thật của thẻ; `flush()` có thật sự cập nhật kích thước file mỗi 250 ms không (rút pin giữa chừng rồi xem file còn bao nhiêu); thư viện SD kiểu SPI chờ bận trong lúc truyền — có làm chậm task app trên nhân 0 không; chế độ SDMMC 1 bit với các chân đã chọn; stack của task `wb_link` (12 kB) và `wb_sd` (8 kB) có đủ cho FatFs không.
- PSRAM: `ps_malloc(512 kB)` thành công trên N16R8 (báo cáo khởi động có dòng `buffer 512 kB`).
