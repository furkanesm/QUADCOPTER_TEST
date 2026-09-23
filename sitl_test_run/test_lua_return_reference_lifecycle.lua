-- ============================================================================
-- S500 Lua Dönüş Referansı Yaşam Döngüsü ve Oturum İzolasyonu Birim Testi
-- ============================================================================

local test_name = "Lua Return Reference Lifecycle & Session Isolation Unit Tests"
print("============================================================================")
print("BAŞLATILIYOR: " .. test_name)
print("============================================================================")

package.path = "/workspace/libraries/AP_Scripting/modules/?.lua;/workspace/sitl_test_run/scripts/modules/?.lua;" .. package.path

local mavlink_msgs = require("MAVLink/mavlink_msgs")

local mock_time_ms = 1000
local mock_is_armed = false
local mock_vehicle_mode = 4 -- GUIDED
local mock_cur_pos_x = 10.0
local mock_cur_pos_y = 20.0
local mock_cur_pos_z = -0.5
local mock_gcs_logs = {}

_G._TEST_ENV = true
_G.millis = function() return mock_time_ms end

_G.MAV_SEVERITY = {
    INFO = 6,
    WARNING = 4,
    ERROR = 3
}

_G.gcs = {
    send_text = function(self, sev, txt)
        table.insert(mock_gcs_logs, { sev = sev, text = txt })
    end
}

_G.Vector3f = function()
    local v = { _x = 0, _y = 0, _z = 0 }
    function v:x(val) if val then self._x = val end return self._x end
    function v:y(val) if val then self._y = val end return self._y end
    function v:z(val) if val then self._z = val end return self._z end
    return v
end

_G.arming = {
    is_armed = function() return mock_is_armed end,
    arm = function() mock_is_armed = true return true end,
    disarm = function() mock_is_armed = false return true end
}

_G.vehicle = {
    get_mode = function() return mock_vehicle_mode end,
    set_mode = function(self, m) mock_vehicle_mode = m return true end,
    start_takeoff = function(self, alt) return true end,
    set_target_pos_NED = function(self, pos, pos_valid, yaw, yaw_valid, yaw_rate, yaw_rate_valid, reset) return true end,
    get_likely_flying = function() return mock_is_armed end
}

_G.ahrs = {
    get_relative_position_NED_origin = function()
        local v = _G.Vector3f()
        v:x(mock_cur_pos_x)
        v:y(mock_cur_pos_y)
        v:z(mock_cur_pos_z)
        return v
    end,
    get_velocity_NED = function()
        local v = _G.Vector3f()
        v:x(0.0) v:y(0.0) v:z(0.0)
        return v
    end,
    get_yaw = function() return 0.0 end,
    get_yaw_rad = function() return 0.0 end
}

_G.param = {
    get = function(self, name) return 1.0 end,
    set = function(self, name, val) return true end
}

local mock_rx_queue = {}
_G.mavlink = {
    init = function(self, d, c) end,
    register_rx_msgid = function(self, id) end,
    block_command = function(self, cmd) end,
    receive_chan = function(self)
        if #mock_rx_queue > 0 then
            local pkt = table.remove(mock_rx_queue, 1)
            return pkt, 0
        end
        return nil, 0
    end,
    send_chan = function(self, c, id, p) return true end
}

local mission = dofile("/workspace/s500_lua_mission/s500_mission.lua")
assert(type(mission) == "table", "s500_mission.lua yuklenemedi!")

-- Tam MAVLink 2 C struct frame üretici
local function make_c_frame(p1, p2, p3, p4, p5, p6, p7, cmd, tgt_sys, tgt_comp, conf, src_sys, src_comp, seq_id)
    cmd = cmd or 31010
    tgt_sys = tgt_sys or 1
    tgt_comp = tgt_comp or 1
    conf = conf or 0
    src_sys = src_sys or 1
    src_comp = src_comp or 191
    seq_id = seq_id or 1

    local payload = string.pack("<fffffffI2BBB",
        p1 or 0.0, p2 or 0.0, p3 or 0.0, p4 or 0.0, p5 or 0.0, p6 or 0.0, p7 or 1.0,
        cmd, tgt_sys, tgt_comp, conf)

    local header_without_crc = string.pack("<BBBBBB", #payload, 0, 0, seq_id, src_sys, src_comp) .. string.pack("<I3", 76)
    local crc_buf = header_without_crc .. payload .. string.char(152) -- COMMAND_LONG crc_extra = 152
    local crc = mavlink_msgs.generateCRC(crc_buf)
    return string.pack("<HB", crc, 0xFD) .. header_without_crc .. payload
end

local function make_session_start_packet(sess_id, seq)
    return make_c_frame(0.0, 0.0, 0.0, seq or 1.0, 0xAC, sess_id, 1.0, 31010, 1, 1, 0, 1, 191, seq or 1)
end

local total = 0
local passed = 0

local function check(desc, cond, extra)
    total = total + 1
    if cond then
        passed = passed + 1
        print(string.format("  [PASS] Test %d: %s", total, desc))
    else
        print(string.format("  [FAIL] Test %d: %s (%s)", total, desc, extra or ""))
        os.exit(1)
    end
end

-- ============================================================================
-- TEST 1: Mevcut hazırlığın geçerli referansı doğru oturumla yeniden yayımlanır
-- ============================================================================
print("\n--- TEST 1: Mevcut Hazırlık Referansının Oturum Açılınca Doğru Yayımlanması ---")
mock_gcs_logs = {}
mock_is_armed = false
mission.set_state(2) -- STATE_HAZIRLIK
mission.set_BASLANGIC_KONUMU({ kuzey = 10.0, dogu = 20.0, z = -0.5 })
mission.set_takeoff_return_point_ned({ x = 10.0, y = 20.0, z = -0.5 })
mission.set_hazirlik_verified(true)

-- Oturum açma paketi gelsin (Session ID: 1001)
table.insert(mock_rx_queue, make_session_start_packet(1001, 1))
mission.process_mavlink_queue(mock_time_ms)

check("Session ID 1001 aktif olmali", mission.get_active_session_id() == 1001)

local found_baslangic = false
local found_takeoff = false
for _, entry in ipairs(mock_gcs_logs) do
    if string.find(entry.text, "BASLANGIC_KONUMU kilitlendi (Session ID: 1001): Kuzey=10.00m, Dogu=20.00m, Z=-0.50m", 1, true) then
        found_baslangic = true
    end
    if string.find(entry.text, "takeoff_return_point_ned kilitlendi (Session ID: 1001): [10.00, 20.00, -0.50]", 1, true) then
        found_takeoff = true
    end
end
check("BASLANGIC_KONUMU Session ID 1001 ile yeniden yayimlanmali", found_baslangic)
check("takeoff_return_point_ned Session ID 1001 ile yeniden yayimlanmali", found_takeoff)

-- ============================================================================
-- TEST 2: Önceki görevden kalan referans yeni oturumla yayımlanmaz
-- ============================================================================
print("\n--- TEST 2: Önceki Görevden Kalan Referansın Yeni Oturumla Yayımlanmaması ---")
mock_gcs_logs = {}
mock_is_armed = false
mission.set_state(1) -- STATE_BEKLEME
mission.set_hazirlik_verified(false) -- Önceki görev bitti / yeni hazırlık doğrulanmadı!
mission.set_BASLANGIC_KONUMU({ kuzey = 99.0, dogu = 88.0, z = 0.0 })
mission.set_takeoff_return_point_ned({ x = 99.0, y = 88.0, z = 0.0 })

-- Yeni oturum açma paketi gelsin (Session ID: 2002)
table.insert(mock_rx_queue, make_session_start_packet(2002, 1))
mission.process_mavlink_queue(mock_time_ms)

check("Session ID 2002 aktif olmali", mission.get_active_session_id() == 2002)
local published_stale_baslangic = false
local published_stale_takeoff = false
for _, entry in ipairs(mock_gcs_logs) do
    if string.find(entry.text, "BASLANGIC_KONUMU kilitlendi (Session ID: 2002)", 1, true) then
        published_stale_baslangic = true
    end
    if string.find(entry.text, "takeoff_return_point_ned kilitlendi (Session ID: 2002)", 1, true) then
        published_stale_takeoff = true
    end
end
check("hazirlik_verified=false iken eski BASLANGIC_KONUMU Session ID 2002 ile YAYIMLANMAMALI", not published_stale_baslangic)
check("hazirlik_verified=false iken eski takeoff_return_point_ned Session ID 2002 ile YAYIMLANMAMALI", not published_stale_takeoff)

-- ============================================================================
-- TEST 3: Uçuş durumunda oturum isteği mevcut dönüş referansını silmez (korur)
-- ============================================================================
print("\n--- TEST 3: Uçuş Sırasında Gelen Oturum İsteğinde Dönüş Referansı Korunması ---")
mock_gcs_logs = {}
mock_is_armed = true -- Araç havada!
mission.set_state(5) -- STATE_HEDEFE_GIT
mission.set_hazirlik_verified(true)
mission.set_BASLANGIC_KONUMU({ kuzey = 10.0, dogu = 20.0, z = -0.5 })
mission.set_takeoff_return_point_ned({ x = 10.0, y = 20.0, z = -0.5 })

-- Uçuş sırasında oturum açma paketi gelsin (Session ID: 3003)
table.insert(mock_rx_queue, make_session_start_packet(3003, 1))
mission.process_mavlink_queue(mock_time_ms)

local ref_after = mission.get_takeoff_return_point_ned()
local base_after = mission.get_BASLANGIC_KONUMU()

check("Ucus sirasinda oturum istegi sonrasinda takeoff_return_point_ned SILINMEMELI", ref_after ~= nil)
check("takeoff_return_point_ned koordinatlari degismeden korunmali",
    ref_after and ref_after.x == 10.0 and ref_after.y == 20.0 and ref_after.z == -0.5)
check("BASLANGIC_KONUMU koordinatlari degismeden korunmali",
    base_after and base_after.kuzey == 10.0 and base_after.dogu == 20.0 and base_after.z == -0.5)

local published_in_flight = false
local reason_logged = false
for _, entry in ipairs(mock_gcs_logs) do
    if string.find(entry.text, "Session ID: 3003", 1, true) and
       (string.find(entry.text, "BASLANGIC_KONUMU kilitlendi", 1, true) or string.find(entry.text, "takeoff_return_point_ned kilitlendi", 1, true)) then
        published_in_flight = true
    end
    if string.find(entry.text, "Mevcut donus referansi korundu ancak yeni oturum icin yeniden yayimlanmadi", 1, true) then
        reason_logged = true
    end
end
check("Ucus sirasinda donus referansi yeni oturum icin yayimlanmamali", not published_in_flight)
check("Yeniden yayimlanmama gerekcesi ve referansin korundugu loglanmali", reason_logged)

-- ============================================================================
-- TEST 4: Eksik referansta çökme olmaması ve oturumun işlenip referans yayınının yapılmaması
-- ============================================================================
print("\n--- TEST 4: Eksik Referansta Güvenli İşleme ve Yayım Yapılmaması ---")
mock_gcs_logs = {}
mock_is_armed = false
mission.set_state(1) -- STATE_BEKLEME
mission.set_hazirlik_verified(true)
mission.set_BASLANGIC_KONUMU({ kuzey = 10.0, dogu = 20.0, z = -0.5 })
mission.set_takeoff_return_point_ned(nil) -- takeoff_return_point_ned eksik!

table.insert(mock_rx_queue, make_session_start_packet(4004, 1))
local ok_run, err = pcall(function() mission.process_mavlink_queue(mock_time_ms) end)
check("takeoff_return_point_ned == nil iken pcall hatasiz tamamlanmali", ok_run, err)
check("Session ID 4004 basariyla islenmis ve aktif olmali", mission.get_active_session_id() == 4004)

local published_when_missing = false
for _, entry in ipairs(mock_gcs_logs) do
    if string.find(entry.text, "Session ID: 4004", 1, true) and
       (string.find(entry.text, "BASLANGIC_KONUMU kilitlendi", 1, true) or string.find(entry.text, "takeoff_return_point_ned kilitlendi", 1, true)) then
        published_when_missing = true
    end
end
check("Eksik referansta hiçbir kilitli referans mesaji yayimlanmamali", not published_when_missing)

-- ============================================================================
-- TEST 5: Gerçek Görev Tetikleme & Hazırlık Döngüsü ile Sıfırlama ve İki Referansın Birlikte Kilitlenmesi
-- ============================================================================
print("\n--- TEST 5: Gerçek Görev Tetikleme Döngüsü (Reset ve İki Referansın Birlikte Kilitlenmesi) ---")
mock_gcs_logs = {}
mock_is_armed = false
mock_vehicle_mode = 0 -- STABILIZE

-- Önceki görevden kalan kirli veriler
mission.set_state(1) -- STATE_BEKLEME
mission.set_hazirlik_verified(true)
mission.set_BASLANGIC_KONUMU({ kuzey = 888.0, dogu = 999.0, z = 111.0 })
mission.set_takeoff_return_point_ned({ x = 888.0, y = 999.0, z = 111.0 })

-- 1. GUIDED moda geçilerek görev tetiklensin
mock_vehicle_mode = 4 -- COPTER_MODE_GUIDED
mock_time_ms = 2000
mission.update() -- STATE_BEKLEME -> STATE_HAZIRLIK geçişini çalıştır

check("Gorev tetiklendiginde STATE_HAZIRLIK (State=2) durumuna gecilmeli", mission.get_current_state() == 2)
check("Yeni hazirlik baslangicinda eski BASLANGIC_KONUMU sifirlanmali (nil)", mission.get_BASLANGIC_KONUMU() == nil)
check("Yeni hazirlik baslangicinda eski takeoff_return_point_ned sifirlanmali (nil)", mission.get_takeoff_return_point_ned() == nil)
check("Yeni hazirlik baslangicinda hazirlik_verified false olmali", mission.get_hazirlik_verified() == false)

-- 2. Yeni zemin konumunda 5 örnek toplanarak hazırlık tamamlansın
mock_cur_pos_x = 15.0
mock_cur_pos_y = 25.0
mock_cur_pos_z = -1.2

-- 5 örnek toplanana kadar update() çağır
for i = 1, 5 do
    mock_time_ms = mock_time_ms + 100
    mission.update()
end

local final_bas = mission.get_BASLANGIC_KONUMU()
local final_ref = mission.get_takeoff_return_point_ned()
local verified_now = mission.get_hazirlik_verified()

check("5 ornek sonrasinda hazirlik_verified true olmali", verified_now == true)
check("Yeni BASLANGIC_KONUMU olcumlerden dogru kilitlenmeli (x=15.0, y=25.0, z=-1.2)",
    final_bas ~= nil and math.abs(final_bas.kuzey - 15.0) < 0.01 and math.abs(final_bas.dogu - 25.0) < 0.01 and math.abs(final_bas.z - (-1.2)) < 0.01)
check("Yeni takeoff_return_point_ned ayni zemin olcumlerinden birlikte kilitlenmeli",
    final_ref ~= nil and math.abs(final_ref.x - 15.0) < 0.01 and math.abs(final_ref.y - 25.0) < 0.01 and math.abs(final_ref.z - (-1.2)) < 0.01)

local found_both_logs = false
local found_bas_log = false
local found_ref_log = false
for _, entry in ipairs(mock_gcs_logs) do
    if string.find(entry.text, "BASLANGIC_KONUMU kilitlendi", 1, true) and string.find(entry.text, "Kuzey=15.00m, Dogu=25.00m, Z=-1.20m", 1, true) then
        found_bas_log = true
    end
    if string.find(entry.text, "takeoff_return_point_ned kilitlendi", 1, true) and string.find(entry.text, "[15.00, 25.00, -1.20]", 1, true) then
        found_ref_log = true
    end
end
check("Gercek hazirlik sonrasi her iki referans da GCS logu olarak uretilmeli", found_bas_log and found_ref_log)

print("\n============================================================================")
print(string.format("LUA REFERANS YAŞAM DÖNGÜSÜ TESTLERİ: %d / %d BAŞARILI | 0 BAŞARISIZ", passed, total))
print("============================================================================")
os.exit(0)
