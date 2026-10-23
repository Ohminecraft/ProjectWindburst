# App: configurator đồ họa, thư viện client, dòng lệnh — `app/`

## Configurator đồ họa (GUI)

```
cd app && pip install -r requirements.txt      # chỉ cần pyserial
python -m wbclient gui                         # mở trình duyệt tại http://127.0.0.1:8765/ ; chọn cổng COM trong trang
python -m wbclient --port COM5 gui             # nối luôn vào COM5
python -m wbclient gui --http 9000 --no-browser
python -m wbclient gui --logs D:\\logs               # thư mục log blackbox (mặc định ~/Windburst/logs)
```
Không có mạch: `make sim && build/wb_sim`, rồi bấm "Tìm cổng" và chọn "Mô phỏng wb_sim".

**Cấu trúc.** GUI là một trang web chạy cục bộ, không cần Internet, không có bước build, không dùng thư viện ngoài:

```
trình duyệt  ──HTTP + server-sent events──  wbclient/gui.py  ──WbClient (WBP)──  USB serial / wb_sim
wbclient/web/index.html, app.css
wbclient/web/js/main.js        thanh kết nối, dòng "vì sao chưa arm được", nút Lưu, chuyển tab
wbclient/web/js/store.js       schema, settings (đúng như firmware đang giữ trong RAM), telemetry, đăng ký stream
wbclient/web/js/ui.js          widget gắn với một field của schema (số / enum / công tắc), nút lệnh
wbclient/web/js/labels.js      nhãn tiếng Việt, đơn vị, giải thích; tên blocker; thứ firmware chưa hỗ trợ
wbclient/web/js/i18n.js        tr('chuỗi tiếng Việt', {giá trị}) → chuỗi theo ngôn ngữ đang chọn
wbclient/web/js/lang/en.js     bản tiếng Anh: EN (Việt → Anh) và SAME (chuỗi giống nhau ở hai ngôn ngữ)
wbclient/web/js/tabs/*.js      mỗi tab một file
wbclient/web/js/gfx/           fwmath.js (công thức giống firmware), craft3d.js (hình 3D), stickpreview.js (máy bay xem thử theo cần), scope.js (đồ thị cuộn), logplot.js (đồ thị log blackbox)
wbclient/logview.py            thư mục log trên máy tính, khung nhìn rút gọn của một log cho đồ thị (xem docs/BLACKBOX.md)
```
- Trình duyệt không thấy giao thức nhị phân: settings và telemetry tới dưới dạng JSON, đơn vị kỹ thuật, enum bằng tên (đúng các giá trị dòng lệnh in ra).
- Sửa một ô là gửi ngay vào **RAM** của firmware (0,2 s sau), và trang hiện lại **giá trị firmware thực sự nhận** (đã kẹp miền). Nút "Lưu" hiện khi firmware báo `DIRTY`; đổi mục cần khởi động lại thì nút thành "Lưu và khởi động lại". "Bỏ thay đổi" = khởi động lại không lưu.
- Đang ARM: mọi ô và nút lệnh bị khóa (firmware cũng từ chối).
- Flight controller tự khởi động lại (nút reset, sụt nguồn) → `gui.py` thấy im lặng, hỏi lại, đăng ký lại stream; trang đọc lại settings. Rút cáp → trang về màn hình kết nối kèm lý do.
- Muốn làm app Electron sau này: bọc đúng trang này (Electron chạy `python -m wbclient gui --no-browser` rồi mở URL), hoặc viết lại `gui.py` bằng Node. Giao diện không phụ thuộc Python.

**Hai ngôn ngữ (Việt / Anh).** Ô chọn ở cuối thanh tab; mặc định theo ngôn ngữ của trình duyệt, lựa chọn được nhớ lại. Đổi ngôn ngữ thì trang tải lại (vẫn giữ kết nối với mạch).
- Chuỗi nguồn trong mã là tiếng Việt, luôn đi qua `tr()`. Bản tiếng Anh nằm ở một file: `web/js/lang/en.js`.
- **Sửa hay thêm một dòng chữ**: viết `tr('Chữ tiếng Việt {n}', { n })` trong mã, thêm một dòng `"Chữ tiếng Việt {n}": "English text {n}"` vào `EN` (hoặc vào `SAME` nếu hai ngôn ngữ viết như nhau: `Hz`, `Roll`, `DShot600`...). Quên thì `test_gui_i18n.py` báo đúng chuỗi nào thiếu, chuỗi nào thừa, `{n}` nào lệch.
- **Thêm ngôn ngữ thứ ba**: thêm file `lang/xx.js` cùng dạng, khai báo trong `LANGS` của `i18n.js`.
- `gui.py` không chứa câu chữ nào cho người dùng: lỗi và trạng thái đi dưới dạng **mã + giá trị** (`{"error": "OPEN_FAILED", "port": ..., "detail": ...}`, `note: {"code": "PORT_LOST", "port": ...}`), trang ghép thành câu theo ngôn ngữ đang chọn. Chữ do firmware gửi (báo cáo khởi động, log) vốn là tiếng Anh và được hiện nguyên văn.
- Tài liệu trong `docs/` vẫn là tiếng Việt.

**Kết nối serial.** Trang nối với mạch qua cổng serial của máy (COM trên Windows, `/dev/ttyACM*` / `ttyUSB*` trên Linux) bằng pyserial trong `gui.py`; `socket://` chỉ dùng cho mô phỏng.
- Danh sách cổng có mô tả của hệ điều hành; cổng có chip hay gặp trên mạch ESP32 (USB gốc Espressif, CP210x, CH34x / CH910x, FTDI) được xếp lên đầu và chọn sẵn. Danh sách tự làm mới mỗi 2,5 s khi chưa kết nối, nên cắm mạch vào là thấy. "Nhập cổng khác…" để gõ tên cổng bất kỳ.
- Ô **baud** cạnh ô cổng (115200 / 230400 / 460800 / 921600, mặc định 230400 = `WB_LINK_BAUD` trong `main.cpp`). Chỉ có ý nghĩa với cầu USB-UART; USB gốc của ESP32-S3 thì baud nào cũng chạy. Lựa chọn được nhớ lại.
- **Mạch không trả lời** (sai cổng, sai baud, chưa nạp firmware, firmware treo vòng lặp khởi động): trang nói rõ từng khả năng và in ra **những gì cổng đã gửi về** (log khởi động của ESP32, "Guru Meditation", hay toàn ký tự lạ khi sai baud). Không mở được cổng: gợi ý đóng serial monitor đang giữ cổng.
- **Một cổng chỉ một chương trình.** Client mở cổng ở chế độ độc quyền (`exclusive` của pyserial) nên trên Linux / macOS cũng như Windows: cổng đang bị serial monitor hay cửa sổ configurator khác giữ thì báo "không mở được", không dùng chung rồi mỗi bên đọc mất một nửa câu trả lời. Kết nối thất bại thì cổng được nhả ngay (để thử lại, hoặc để nạp firmware).
- **Dữ liệu thô** (tab Hệ thống): mọi byte không thuộc khung WBP trong lúc đang kết nối được tách riêng và hiện dưới dạng chữ, ví dụ firmware in lỗi rồi khởi động lại. Liên kết vẫn chạy bình thường vì parser tự đồng bộ lại.
- Rút cáp: trang về màn hình kết nối và ghi "Mất kết nối với cổng …".

**Các tab**

| Tab | Có gì | Firmware chưa có (trang ghi rõ) |
|---|---|---|
| Tổng quan | hình 3D theo tư thế, bảng module (có / không có + lý do firmware báo; thêm dòng "Optical flow" khi đã chọn cảm biến và dòng "Bảng chân" khi firmware thấy trùng chân), danh sách lý do chưa arm được (kể cả "vòng điều khiển không đạt tần số", "vừa bị tắt vì runaway"), số đo trực tiếp, **CPU / RAM / PSRAM**, **tần số thực / tần số đặt và số chu kỳ bỏ lỡ, thời gian từng khâu của một chu kỳ, failsafe kèm nguyên nhân** | |
| Receiver | 12 kênh trực tiếp theo channel map, chất lượng sóng, giao thức, thứ tự kênh (AETR / TAER), RC smoothing, máy bay xem thử theo cần (mặc định Angle) | SPEKTRUM, nRF24, PPM, PWM |
| Mode | dải AUX cho từng mode (kéo hai đầu dải; vạch đậm = vị trí công tắc lúc này), trạng thái bật/tắt thật; có Position Hold và GPS Rescue | |
| Cảm biến | hiệu chuẩn gyro, accelerometer 6 mặt, level trim, **level trim khi bay** (trạng thái lần đo: chưa đo / đang đo % / đã đo xong chờ disarm; trang tự đọc lại và báo khi firmware tự lưu), la bàn (đám mây điểm); hướng chip; airmode + ngưỡng ga bắt đầu; bật tắt module; GPS + đặt home | gyro 1/2 |
| Motor | hình khung với số motor và chiều quay bắt buộc, giao thức ESC (motor idle chỉ hiện với DShot), test từng motor (RPM, lỗi telemetry), **dòng trạng thái DShot hai chiều: ESC nào không trả lời, khung bỏ qua, lượt trả lời bị lỡ**, đảo chiều, thứ tự motor, đồ thị rung gyro thô / sau lọc | DShot quá 4 motor |
| PID | **thanh trượt tune** (kéo: chỉ con số cạnh thanh đổi; thả: một lần ghi; firmware tính P I D Dmax F và trả về, ô của trục do thanh trượt tính bị khoá), bảng PID từng trục, feedforward, I-term relax, anti-gravity, D max, TPA và throttle boost, Angle, Stabilize (P từng trục, gia tốc, input time constant); số liệu lúc này: TPA, anti-gravity, airmode, PID sum, **ga treo đo được** | |
| Rate | rate kiểu Actual + đồ thị có chấm vị trí cần, máy bay xem thử theo cần (mặc định Acro), đường cong ga, giới hạn góc của Angle / Stabilize | |
| Bộ lọc | **bản đồ tần số** (từng lowpass và vùng nó trượt, notch tĩnh, vị trí dynamic notch từng trục và RPM notch từng motor lúc này), hai thanh trượt lọc (bật thì các ô tần số cắt bị khoá và hiện kết quả), gyro lowpass 1 / 2, notch tĩnh, RPM filter (ghi rõ khi DShot hai chiều chưa bật), dynamic notch, lọc D-term, yaw lowpass | |
| **Vị trí** | cảm biến optical flow: loại, dòng firmware báo lúc khởi động, số mẫu/giây, chất lượng bề mặt, lý do mẫu bị bỏ, tốc độ đo được và **vòng tròn có mũi tên** để kiểm tra hướng lắp (mũi máy ở phía trên; hướng lắp / lật gương / hệ số có tác dụng ngay); ước lượng vị trí (nguồn vận tốc, vị trí, hướng mũi theo la bàn hay chỉ gyro, góc nghiêng có được chỉnh hay không); Position Hold (trạng thái, từng trục, góc nghiêng yêu cầu, điểm giữ, bù gió, hệ số); GPS Rescue và thủ tục failsafe (sẵn sàng hay chưa, giai đoạn + lý do bỏ cuộc, khoảng cách tới home, độ cao bay về, mọi thông số); xem `docs/NAV.md` | |
| Pin và còi | điện áp, số cell, thang một cell theo ngưỡng, các ngưỡng, hiệu chỉnh hệ số chia ADC; bảng 18 tình huống với **hai cột: còi và motor**, hình mẫu âm, số tiếng motor phát, nút "Thử"; âm của motor; loại còi (xem `docs/POWER.md`) | đo dòng điện, mAh |
| Blackbox | chọn nơi ghi (**flash / thẻ SD**) / điều kiện / tốc độ / nhóm trường; kiểu nối và tốc độ thẻ; bộ nhớ log (tải về, xoá, mức đầy bộ đệm thẻ); ghi trực tiếp qua cáp; thư viện log trên máy; **đồ thị** nhiều khung chung trục thời gian, thu phóng, đọc giá trị ở con trỏ, sự kiện, xuất CSV. Xem log không cần nối mạch (xem `docs/BLACKBOX.md`) | ổ USB (mass storage) |
| Chân và cổng | GPIO từng module (kể cả ADC pin, còi, thẻ SD, **optical flow**: chân của loại cảm biến không được chọn thì không tính là trùng, như firmware), cảnh báo chân trùng / chân strapping / USB / flash / PSRAM của ESP32-S3, chân không đọc được ADC; nút **"Điền bộ chân gợi ý"** cho DevKitC N16R8 (xem `docs/BOARD_N16R8.md`) | đổi chức năng UART |
| Hệ thống | failsafe (thời gian chờ, thời gian sóng phải ổn định lại, giới hạn khi không xuống được), **chặn runaway lúc cất cánh**, sao lưu / khôi phục, về mặc định, nhật ký firmware, dữ liệu thô của cổng | |

Thanh trên cùng: khi failsafe đang chạy thì ghi "Failsafe: đang tự hạ cánh (mất sóng)" (giai đoạn + nguyên nhân); ô "Vòng lặp" hiện `thực / đặt Hz` và đổi màu khi dưới 80 %.
Tab Blackbox: thông tin log có "thực tế … bản ghi/giây" khi log thưa hơn cấu hình; bấm tải log lúc đang ghi trực tiếp thì phiên ghi tự dừng (file được giữ) rồi mới tải.

Chưa có: tab cho TX.

Cách tune và ý nghĩa từng thanh trượt: `docs/TUNING.md`.

**An toàn của server cục bộ** (nó quay được motor, nên không được nghe lời bất kỳ trang web nào đang mở): chỉ nghe trên `127.0.0.1`; mọi `/api` phải kèm token ngẫu nhiên nhúng trong trang do chính nó phục vụ; kiểm tra `Host` và `Origin` (chặn DNS rebinding và request từ site khác); POST phải là `application/json`.

**Test motor**: thanh trượt bị khóa cho tới khi tick "Đã tháo hết cánh quạt"; trang gửi lại lệnh mỗi 0,2 s và firmware tự dừng motor nếu 0,6 s không nhận được, nên trang treo hay rút cáp thì motor dừng; rời tab hoặc ẩn cửa sổ cũng dừng.

**Hình vẽ không được lệch khỏi firmware**: `tools/ui_reference.cpp` in ra hàng mixer, đường cong rate, đường cong ga, deadband do **mã firmware** tính; `app/tests/test_gui_math.py` so với `js/gfx/fwmath.js` (chạy bằng node).

## Thư viện client và dòng lệnh

```
app/wbclient/framing.py     khung WBP + parser tự đồng bộ (bản Python của wbp_codec.h)
app/wbclient/schema.py      đọc protocol/wbp_schema.json: layout, scale, enum, bit → giá trị "người đọc được"
app/wbclient/client.py      WbClient: hello, get/set/save/defaults/reboot, subscribe, command, report, blackbox_*
app/wbclient/wblog.py       đọc log blackbox (.wblog): docs/BLACKBOX.md
app/wbclient/__main__.py    python -m wbclient ...  (configurator dòng lệnh, và lệnh `gui`)
app/wbclient/gui.py         server cục bộ của GUI;  app/wbclient/web/  trang web
app/tests/                  pytest: khung, schema, và test đầu-cuối với build/wb_sim
app/requirements.txt        pyserial
```

## Thư viện

```python
from wbclient import WbClient, WbError

with WbClient("COM5") as fc:                      # hoặc "/dev/ttyACM0", hoặc "socket://127.0.0.1:5760" (wb_sim)
    print(fc.info)                                # tên, phiên bản, role, schema hash, features
    print(fc.report())                            # cảm biến nào tìm thấy / vì sao không

    pid = fc.get("pid")                           # dict, đơn vị kỹ thuật
    stored = fc.set("pid", p=[0.05, 0.05, 0.012]) # trả về giá trị firmware THỰC SỰ lưu (đã kẹp miền)
    fc.set("motors", protocol="DSHOT600")         # enum bằng tên
    fc.save(); fc.reboot()                        # reboot() chờ tới khi firmware trả lời lại

    fc.subscribe("attitude", 50, lambda v: print(v["roll"], v["pitch"]))
    fc.subscribe("status", 10)
    st = fc.wait_stream("status")                 # {'armed': 0, 'blockers': ['RX_LOST'], 'health': ['IMU', ...], ...}

    try:
        fc.command("MOTOR_TEST", motor=0, value=100)       # 10 %, phải gửi lại ≤ 0,5 s/lần
    except WbError as e:
        print(e.name)                             # 'ARMED', 'BAD_ARG', 'NO_SENSOR', ...
```
- Mở cổng là gửi `HELLO` và so **schema hash**; khác nhau → `SchemaMismatch` (không đọc nhầm settings).
- Một luồng đọc giải mã mọi thứ: trả lời khớp theo `seq`, stream vào `latest()` / callback, log firmware vào `fc.logs`.
- Yêu cầu đọc/ghi settings được tự gửi lại khi quá hạn; **command thì không** (không lặp lại một lệnh có tác dụng phụ).
- Scale, enum, bit được đổi sẵn: `0.1°` → độ, `blockers` → danh sách tên, ô mode trống → `None`.

## Dòng lệnh

```
cd app && pip install -r requirements.txt
python -m wbclient --port COM5 info
python -m wbclient --port COM5 get pid
python -m wbclient --port COM5 set pins imu_cs=10 imu_miso=13 imu_mosi=11 imu_sck=12 motor=4,5,6,7,255,255,255,255
python -m wbclient --port COM5 set modes mode[0]=ARM aux[0]=0 lo[0]=1700 hi[0]=2100            # ô mode số 0: ARM khi AUX1 ở 1700..2100
python -m wbclient --port COM5 set modes mode[1]=ANGLE aux[1]=1 lo[1]=1700 hi[1]=2100
python -m wbclient --port COM5 save
python -m wbclient --port COM5 reboot
python -m wbclient --port COM5 monitor attitude,status,rc --hz 20
python -m wbclient --port COM5 calibrate gyro | level | acc | mag
python -m wbclient --port COM5 motor 0 10 --seconds 2 --props-off
python -m wbclient --port COM5 backup my_quad.json      /   restore my_quad.json
python -m wbclient --port COM5 blackbox status | list | download [N] | erase | record FILE     # docs/BLACKBOX.md
python -m wbclient blackbox info FILE | csv FILE        # đọc file log, không cần mạch
python -m wbclient schema                               # liệt kê section, stream, command
```
Trường mảng: nhập cả mảng (`motor=4,5,6,7,255,255,255,255`, phải **đủ số phần tử**) hoặc từng phần tử (`motor[2]=6`). Sửa nhiều thứ một lúc: `backup` → sửa file json → `restore`.

Không có mạch: `make sim && build/wb_sim`, rồi dùng `--port socket://127.0.0.1:5760`.

## Kiểm chứng

`make apptest` (`pip install -r app/requirements-dev.txt`; test trình duyệt cần `playwright` + Chromium, test công thức và bản dịch cần `node`, thiếu thì tự bỏ qua):

| File | Số test | Nội dung |
|---|---|---|
| `test_framing.py`, `test_e2e.py` | 29 | khung và parser (nhiễu, header giả, lỗi bit), json khớp header C++, **mặc định của mọi trường = của firmware**, kịch bản đầu-cuối của thư viện và CLI với firmware thật trong `wb_sim`; pin (ADC, ngưỡng live), còi và im lặng trên bàn; từ 0.7.0: cảm biến optical flow (cầm máy đi, hướng lắp, Position Hold trong gió) và GPS Rescue từ đầu đến khi hạ cánh |
| `test_blackbox.py` | 8 | trình đọc `.wblog` với log viết tay từng byte (tràn đồng hồ, khung hỏng, file cụt), bay → flash mô phỏng → tải → giải mã → so với máy ảo, luồng trực tiếp, dữ liệu lạ trong phân vùng, CLI |
| `test_logview.py` | 5 | đọc log đang lớn dần = đọc một lần; khung nhìn rút gọn giữ mọi cực trị; thư mục log (tên file an toàn, không ghi đè, file đổi thì đọc lại); ghi trực tiếp vừa ra file vừa ra đồ thị |
| `test_gui_api.py` | 18 | server của GUI và lệnh `gui`: token / Host / Origin, lỗi dạng mã + giá trị, set + kẹp + lưu + reboot, stream, từ chối khi ARM, firmware tự reset, rút cáp, sao lưu / khôi phục; **serial qua thiết bị thật của hệ điều hành** (pseudo-terminal nối với `wb_sim`): baud, telemetry, chữ lạ xen giữa các khung, rút cáp, cổng không trả lời (có / không có dữ liệu) và được nhả sau đó, cổng đang bị chương trình khác giữ; **log blackbox**: tải từ mạch có tiến độ, một lúc một việc, khung nhìn rút gọn, CSV, settings của chuyến bay, xoá / thêm file, ghi trực tiếp và đồ thị bám theo, từ chối khi đang bay |
| `test_gui_store.py` | 1 | kho settings của trang (chạy bằng node): hai lần sửa liên tiếp của cùng một section tới firmware đúng thứ tự, không cái nào mất |
| `test_gui_math.py` | 7 | công thức của trang = số do firmware tính (hàng mixer 3 kiểu khung, rate, ga, deadband, phép quay của hình 3D), mẫu âm của còi |
| `test_gui_i18n.py` | 5 | mọi chuỗi của trang có bản tiếng Anh (hoặc nằm trong `SAME`), không có mục thừa, `{giá trị}` khớp, không chuỗi tiếng Việt nào nằm ngoài `tr()`; **mọi blocker / nguyên nhân failsafe / trạng thái driver mà firmware có thể báo đều có chữ** |
| `test_gui_browser.py` | 25 | trang thật trong Chromium không đầu + firmware thật trong `wb_sim`: kết nối, sửa / kẹp / lưu / mất điện / khởi động lại / bỏ thay đổi, tab Mode, test motor (đo ở ESC ảo), đồ thị rung, hiệu chuẩn accelerometer + level + la bàn, tab chân, khóa khi ARM, mất cổng, khôi phục / mặc định; **trang tiếng Anh: cả 13 tab và các trạng thái đặc biệt không còn chữ Việt nào**, đổi ngôn ngữ được nhớ; kết nối qua cổng serial từ trang, giải thích khi cổng không trả lời; tab Pin và còi (ngưỡng, hiệu chỉnh ADC, còi đo ở chân GPIO ảo); **tab Blackbox**: cấu hình → bay → tải → đồ thị, giá trị ở con trỏ và vị trí nét vẽ so với log, thu phóng, sự kiện, xuất, xoá, ghi trực tiếp, xem log không nối mạch; **tab Vị trí**: báo cáo của firmware về cảm biến, mũi tên tốc độ khi cầm máy đi (đo bằng điểm ảnh), sửa hướng lắp không cần khởi động lại, Position Hold trong gió, rescue bị từ chối khi không có la bàn, chân của cảm biến flow |

Mỗi test trình duyệt còn kiểm tra trang không ghi lỗi nào ra console. Mutation test (`python3 tools/mutate_gui.py`): xem con số trong `docs/HANDOFF.md`.

**Chưa kiểm chứng**:
- Cổng COM trên Windows và mạch ESP32-S3 thật (USB CDC gốc hoặc cầu USB-UART). Đường serial mới được chạy qua pseudo-terminal trên Linux: cùng mã pyserial, nhưng không có driver USB, không có baud thật, không có DTR/RTS. Chế độ độc quyền trên Windows dựa vào hành vi sẵn có của COM, chưa thử. Việc giữ DTR/RTS thấp có ngăn được mạch reset khi mở cổng hay không vẫn chưa biết.
- Nhận diện cổng theo VID của chip USB (danh sách trong `gui.py`) chưa thử với mạch nào.
- GUI mới chỉ chạy trong Chromium không đầu trên Linux. Chưa mở bằng Chrome / Edge / Firefox trên Windows; font Bahnschrift (số) chỉ có trên Windows nên ảnh chụp trong lúc phát triển dùng font thay thế.
- `wb_sim` trên Windows (mã winsock đã viết nhưng chưa biên dịch thử).

## Máy bay xem thử theo cần (0.24.1)
Tab **Receiver** và tab **Rate** có khối "Máy bay theo cần": hình 3D của máy, chạy theo cần đang gạt trên tay điều khiển (stream `rc`). Đây chỉ là mô phỏng trong app, không gửi gì xuống firmware.
- **Acro** (mặc định ở tab Rate): máy quay đúng tốc độ mà đường cong rate cho ra với vị trí cần lúc đó. Thả cần thì máy đứng yên ở tư thế đang có. Cách cộng dồn góc giống bộ ước lượng tư thế của firmware khi chỉ dùng gyro (`acroStep`).
- **Angle** (mặc định ở tab Receiver): góc nghiêng roll / pitch bằng góc đích của `AngleController` (giới hạn góc × rate / max rate). Cần yaw xoay hướng mũi.
- Nút "Về cân bằng" đặt máy về thăng bằng, mũi hướng ra xa người xem. Khi mất sóng thì máy đứng yên và trang báo "Chưa có tín hiệu".
- **Kiểm chứng:** `test_gui_math::test_stick_preview_flies_like_the_firmware` so `acroStep` với `Ahrs` của firmware chỉ dùng gyro (6 đoạn quay nối nhau), và so `angleTarget` với `AngleController` (198 trường hợp, có cả độ nhạy giữa lớn hơn max rate). `test_gui_browser::test_stick_preview` đã viết nhưng chưa chạy được ở máy này.

## Chương trình chạy riêng (cửa sổ riêng, không mở trình duyệt)

`dist/Windburst.exe` (khoảng 14 MB) là một file chạy được ngay. Trong file đã có sẵn Python, wbclient, trang web và schema. Mở lên thì hiện **cửa sổ riêng**, không mở trình duyệt và không có cửa sổ dòng lệnh.

- **Bên trong:** cửa sổ dùng Edge WebView2 của Windows (thư viện pywebview). Windows 11 có sẵn WebView2; Windows 10 có nếu đã cập nhật.
- **Server:** chỉ chạy trên 127.0.0.1, ở một cổng còn trống. Đóng cửa sổ thì server dừng và cổng serial được thả ra.
- **Lưu cài đặt của trang** (theme, ngôn ngữ, baud): nằm ở `%LOCALAPPDATA%\Windburst\webview`, nên mở lại vẫn còn. Log bay vẫn lưu ở chỗ cũ (`~/Windburst/logs`).
- **Chạy từ mã nguồn, không cần build:** `python -m wbclient app` (cần `pip install pywebview`). Lệnh `python -m wbclient gui` vẫn mở bằng trình duyệt như trước.
- **Build lại**, chạy trong thư mục `app`:
  ```
  pip install pywebview pyinstaller pyserial
  python packaging/build_exe.py
  ```
  Kết quả ra ở `app/dist/Windburst.exe`. `dist/` và `build/` không đưa vào git.
- **Icon:** `packaging/windburst.ico`, vẽ bằng `packaging/make_icon.py` (cần Pillow).
