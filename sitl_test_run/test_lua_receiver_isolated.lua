-- ============================================================================
-- S500 Lua Receiver İzole Birim Test Paketi
-- ArduPilot Çalışma Zamanı Mock Ortamı
-- ============================================================================

local test_name = "S500 Lua Receiver Isolated Tests"
print("============================================================================")
print("BAŞLATILIYOR: " .. test_name)
print("============================================================================")

-- 1. Modül Arama Yolu
package.path = "/workspace/libraries/AP_Scripting/modules/?.lua;/workspace/sitl_test_run/scripts/modules/?.lua;" .. package.path

local mavlink_msgs = require("MAVLink/mavlink_msgs")
local msg_entry_cmd_long = require("MAVLink/mavlink_msg_COMMAND_LONG")

-- 2. Mock Durum Değişkenleri
local mock_time_ms = 1000
local mock_is_armed = false
local mock_arm_calls = 0
local mock_takeoff_calls = 0
local mock_vehicle_mode = 4 -- GUIDED
local mock_rx_queue = {}
local mock_tx_packets = {}
local mock_send_chan_result = true

-- 3. ArduPilot Global Mock Nesneleri
_G._TEST_ENV = true
_G.millis = function()
    return mock_time_ms
end

_G.MAV_SEVERITY = {
    INFO = 6,
    WARNING = 4,
    ERROR = 3
}

_G.gcs = {
    send_text = function(self, sev, txt)
        -- print("[MOCK GCS]", sev, txt)
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
    is_armed = function()
        return mock_is_armed
    end,
    arm = function()
        mock_arm_calls = mock_arm_calls + 1
        mock_is_armed = true
        return true
    end,
    disarm = function()
        mock_is_armed = false
        return true
    end
}

_G.vehicle = {
    get_mode = function()
        return mock_vehicle_mode
    end,
    set_mode = function(self, mode)
        mock_vehicle_mode = mode
        return true
    end,
    start_takeoff = function(self, alt)
        mock_takeoff_calls = mock_takeoff_calls + 1
        return true
    end,
    set_target_pos_NED = function(self, pos, yaw, yaw_rate)
        return true
    end
}

_G.ahrs = {
    get_relative_position_NED_origin = function()
        local v = _G.Vector3f()
        v:x(0.0)
        v:y(0.0)
        v:z(-33.0)
        return v
    end,
    get_velocity_NED = function()
        local v = _G.Vector3f()
        v:x(0.0)
        v:y(0.0)
        v:z(0.0)
        return v
    end,
    get_yaw = function() return 0.0 end,
    get_yaw_rad = function() return 0.0 end
}

_G.param = {
    get = function(self, name)
        if name == "WP_SPD" then return 1.0 end
        return 1.0
    end,
    set = function(self, name, val) return true end
}

_G.rc = {
    get_aux_cached = function(self, ch) return 0 end
}

_G.mavlink = {
    init = function(self, depth, count) end,
    register_rx_msgid = function(self, id) end,
    block_command = function(self, cmd) end,
    receive_chan = function(self)
        if #mock_rx_queue > 0 then
            local item = table.remove(mock_rx_queue, 1)
            return item.raw, item.chan
        end
        return nil, 0
    end,
    send_chan = function(self, chan, id, payload)
        table.insert(mock_tx_packets, { chan = chan, id = id, payload = payload })
        return mock_send_chan_result
    end
}

-- 4. Ana Lua Görev Dosyasını Yükle
local mission = dofile("/workspace/s500_lua_mission/s500_mission.lua")
assert(type(mission) == "table", "s500_mission.lua bir tablo dondurmedi!")

-- 5. Test Yardımcı Fonksiyonları
local total_tests = 0
local passed_tests = 0
local failed_tests = 0
local test_results = {}

local function report(name, success, msg)
    total_tests = total_tests + 1
    if success then
        passed_tests = passed_tests + 1
        table.insert(test_results, { name = name, status = "PASS", note = msg or "" })
        print(string.format("  [PASS] %s", name))
    else
        failed_tests = failed_tests + 1
        table.insert(test_results, { name = name, status = "FAIL", note = msg or "Assertion failed" })
        print(string.format("  [FAIL] %s - %s", name, msg or "Assertion failed"))
    end
end

-- Tam MAVLink 2 C struct frame üretici (mavlink_msgs.decode ve binary unpack ile %100 uyumlu)
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

-- Sadece ilgili test adımında yeni gönderilen APP_ACK paketini ayrıştırır
local function get_new_tx_ack(prev_count)
    if #mock_tx_packets <= prev_count then
        return nil
    end
    local pkt = mock_tx_packets[#mock_tx_packets]
    local payload = pkt.payload
    if #payload < 33 then
        return nil
    end
    local p1, p2, p3, p4, p5, p6, p7, cmd, tgt_sys, tgt_comp, conf = string.unpack("<fffffffI2BBB", payload, 1)
    return {
        acked_type  = math.floor(p1 + 0.5),
        result      = math.floor(p2 + 0.5),
        acked_seq   = math.floor(p4 + 0.5),
        status_ack  = math.floor(p5 + 0.5),
        acked_sess  = math.floor(p6 + 0.5),
        proto_ver   = p7,
        command     = cmd,
        target_sys  = tgt_sys,
        target_comp = tgt_comp,
        tx_count    = #mock_tx_packets
    }
end

-- ============================================================================
-- SENARYO 1: Resmî mavlink_msgs modülüyle ayrıştırma yolunun doğrulanması
-- ============================================================================
print("\n[SENARYO 1] Resmî mavlink_msgs Ayrıştırma Yolu ve sysid/compid Doğrulaması...")
local decode_call_count = 0
local decode_last_result = nil
local orig_decode = mavlink_msgs.decode
mavlink_msgs.decode = function(bytes, map)
    decode_call_count = decode_call_count + 1
    decode_last_result = orig_decode(bytes, map)
    return decode_last_result
end

local frame1 = make_c_frame(15.5, -25.0, 0.88, 7, 3, 10001, 1.0, 31010, 1, 1, 0, 1, 191, 1)
decode_call_count = 0
decode_last_result = nil
local parsed1 = mission.parse_command_long(frame1)

local s1_ok = (decode_call_count == 1) and
              (decode_last_result ~= nil) and
              (decode_last_result.sysid == 1) and
              (decode_last_result.compid == 191) and
              (decode_last_result.command == 31010) and
              (parsed1 ~= nil) and
              (parsed1.src_sys == decode_last_result.sysid) and
              (parsed1.src_comp == decode_last_result.compid) and
              (parsed1.target_system == 1) and
              (parsed1.target_component == 1) and
              (parsed1.command == 31010) and
              (math.abs(parsed1.param1 - 15.5) < 0.001) and
              (math.abs(parsed1.param2 - (-25.0)) < 0.001) and
              (math.abs(parsed1.param3 - 0.88) < 0.001) and
              (parsed1.param4 == 7.0) and
              (parsed1.param5 == 3.0) and
              (parsed1.param6 == 10001.0) and
              (parsed1.param7 == 1.0)

report("SENARYO 1: mavlink_msgs.decode ile src_sys/compid ve tum alanlar ayristirildi", s1_ok,
    string.format("decode_calls=%d, src_sys=%s, src_comp=%s, tgt_sys=%s, tgt_comp=%s",
        decode_call_count,
        tostring(parsed1 and parsed1.src_sys), tostring(parsed1 and parsed1.src_comp),
        tostring(parsed1 and parsed1.target_system), tostring(parsed1 and parsed1.target_component)))

-- ============================================================================
-- SENARYO 2: Handshake ve Arm/Kalkış Çağrılmaması (Disarmed Korunması)
-- ============================================================================
print("\n[SENARYO 2] Handshake ve Arm/Kalkış Çağrılmaması...")
mock_is_armed = false
mock_arm_calls = 0
mock_takeoff_calls = 0
mock_tx_packets = {}
mock_time_ms = 2000

local tx_prev_hs = #mock_tx_packets
-- SESSION_START paketi gönder (0xAC = 172, session_id = 12345, seq = 1)
local frame_handshake = make_c_frame(0.0, 0.0, 0.0, 1, 0xAC, 12345, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_handshake, chan = 0 })

local action_hs = mission.process_mavlink_queue(mock_time_ms)
local ack_hs = get_new_tx_ack(tx_prev_hs)

local s2_ok = (mock_arm_calls == 0) and
              (mock_takeoff_calls == 0) and
              (mock_is_armed == false) and
              (action_hs == nil) and
              (mission.get_active_session_id() == 12345) and
              (ack_hs ~= nil and ack_hs.acked_type == 0xAC and ack_hs.result == 0 and ack_hs.acked_seq == 1 and ack_hs.acked_sess == 12345 and ack_hs.status_ack == 0xAB)

report("SENARYO 2: Handshake kabul edildi, ACK(0) gonderildi, arm/kalkis cagrilmadi (is_armed=false)", s2_ok,
    string.format("arm_calls=%d, takeoff_calls=%d, is_armed=%s, sess_id=%s, ack_res=%s",
        mock_arm_calls, mock_takeoff_calls, tostring(mock_is_armed),
        tostring(mission.get_active_session_id()), tostring(ack_hs and ack_hs.result)))

-- ============================================================================
-- SENARYO 3: Geçersiz Alan, Kaynak ve Hedef Reddi (last_msg_time_ms Korunumu)
-- ============================================================================
print("\n[SENARYO 3] Geçersiz Alan, Kaynak ve Hedef Reddi...")
local time_before_invalid = mission.get_last_msg_time_ms()
mock_time_ms = 3000

-- 3a: Eksik kaynak bilgisi (Saf 33 bayt payload, MAVLink header yok -> src_sys ve src_comp nil)
local raw_no_header = string.pack("<fffffffI2BBB", 1.0, 2.0, 0.5, 2, 3, 12345, 1.0, 31010, 1, 1, 0)
table.insert(mock_rx_queue, { raw = raw_no_header, chan = 0 })
local ev_no_hdr = mission.process_mavlink_queue(mock_time_ms)
local time_after_3a = mission.get_last_msg_time_ms()
local s3a_ok = (ev_no_hdr == nil) and (time_after_3a == time_before_invalid)
report("SENARYO 3a: Eksik kaynak (headersiz saf payload) reddedildi, last_msg_time degismedi", s3a_ok)

-- 3b: Yetkisiz kaynak (src_sys = 2, src_comp = 100)
local frame_bad_src = make_c_frame(1.0, 2.0, 0.5, 2, 3, 12345, 1.0, 31010, 1, 1, 0, 2, 100, 1)
table.insert(mock_rx_queue, { raw = frame_bad_src, chan = 0 })
local ev_bad_src = mission.process_mavlink_queue(mock_time_ms + 10)
local s3b_ok = (ev_bad_src == nil) and (mission.get_last_msg_time_ms() == time_before_invalid)
report("SENARYO 3b: Yetkisiz kaynak (sys=2, comp=100) reddedildi", s3b_ok)

-- 3c: Eksik hedef adresi (30 bayt payload, target_system/component ayrıştırılamadı)
local raw_30bytes = string.pack("<fffffffI2", 1.0, 2.0, 0.5, 2, 3, 12345, 1.0, 31010)
table.insert(mock_rx_queue, { raw = raw_30bytes, chan = 0 })
local ev_no_tgt = mission.process_mavlink_queue(mock_time_ms + 20)
local s3c_ok = (ev_no_tgt == nil) and (mission.get_last_msg_time_ms() == time_before_invalid)
report("SENARYO 3c: Eksik hedef adresi (30 bayt) reddedildi", s3c_ok)

-- 3d: Geçersiz hedef adresi (target_system = 2, target_component = 5)
local frame_bad_tgt = make_c_frame(1.0, 2.0, 0.5, 2, 3, 12345, 1.0, 31010, 2, 5, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_bad_tgt, chan = 0 })
local ev_bad_tgt = mission.process_mavlink_queue(mock_time_ms + 30)
local s3d_ok = (ev_bad_tgt == nil) and (mission.get_last_msg_time_ms() == time_before_invalid)
report("SENARYO 3d: Gecersiz hedef adresi (tgt_sys=2, tgt_comp=5) reddedildi", s3d_ok)

-- 3e: Desteklenmeyen protokol sürümü (param7 = 2.0)
local frame_bad_ver = make_c_frame(1.0, 2.0, 0.5, 2, 3, 12345, 2.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_bad_ver, chan = 0 })
local ev_bad_ver = mission.process_mavlink_queue(mock_time_ms + 40)
local s3e_ok = (ev_bad_ver == nil) and (mission.get_last_msg_time_ms() == time_before_invalid)
report("SENARYO 3e: Desteklenmeyen protokol surumu (ver=2.0) reddedildi", s3e_ok)

-- 3f: Geçersiz status (param5 = 99 veya ondalıklı 1.5)
local frame_bad_st = make_c_frame(1.0, 2.0, 0.5, 2, 99.0, 12345, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_bad_st, chan = 0 })
local ev_bad_st = mission.process_mavlink_queue(mock_time_ms + 50)
local s3f_ok = (ev_bad_st == nil) and (mission.get_last_msg_time_ms() == time_before_invalid)
report("SENARYO 3f: Gecersiz status (99.0) reddedildi", s3f_ok)

-- 3g: Geçersiz session_id (> 16777215 veya <= 0)
local frame_bad_sess = make_c_frame(1.0, 2.0, 0.5, 2, 3, 20000000.0, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_bad_sess, chan = 0 })
local ev_bad_sess = mission.process_mavlink_queue(mock_time_ms + 60)
local s3g_ok = (ev_bad_sess == nil) and (mission.get_last_msg_time_ms() == time_before_invalid)
report("SENARYO 3g: Gecersiz session_id (>16777215) reddedildi", s3g_ok)

-- 3h: Geçersiz sequence (> 16777215 veya negatif)
local frame_bad_seq = make_c_frame(1.0, 2.0, 0.5, -5.0, 3, 12345, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_bad_seq, chan = 0 })
local ev_bad_seq = mission.process_mavlink_queue(mock_time_ms + 70)
local s3h_ok = (ev_bad_seq == nil) and (mission.get_last_msg_time_ms() == time_before_invalid)
report("SENARYO 3h: Gecersiz sequence (seq=-5) reddedildi", s3h_ok)

-- 3i: Aralık dışı confidence (<0.0, >1.0 ve hedef olayda == 0.0)
local frame_neg_conf = make_c_frame(1.0, 2.0, -0.2, 2, 3, 12345, 1.0, 31010, 1, 1, 0, 1, 191, 1)
local frame_over_conf = make_c_frame(1.0, 2.0, 1.5, 2, 3, 12345, 1.0, 31010, 1, 1, 0, 1, 191, 1)
local frame_zero_target_conf = make_c_frame(1.0, 2.0, 0.0, 2, 3, 12345, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_neg_conf, chan = 0 })
table.insert(mock_rx_queue, { raw = frame_over_conf, chan = 0 })
table.insert(mock_rx_queue, { raw = frame_zero_target_conf, chan = 0 })
local ev_conf = mission.process_mavlink_queue(mock_time_ms + 80)
local s3i_ok = (ev_conf == nil) and (mission.get_last_msg_time_ms() == time_before_invalid)
report("SENARYO 3i: Aralik disi (-0.2, 1.5) ve hedefte sifir (0.0) confidence reddedildi", s3i_ok)

-- 3j: Ondalıklı Protokol Sürümü (param7 = 1.2 -> yuvarlansa 1.0 olurdu) reddi
local frame_dec_ver = make_c_frame(1.0, 2.0, 0.5, 2, 3, 12345, 1.2, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_dec_ver, chan = 0 })
local ev_dec_ver = mission.process_mavlink_queue(mock_time_ms + 90)
report("SENARYO 3j: Ondalikli surum (1.2) yuvarlanmadan reddedildi, eylem yok, zaman degismedi",
    ev_dec_ver == nil and mission.get_last_msg_time_ms() == time_before_invalid)

-- 3k: Ondalıklı Sequence Numarası (param4 = 2.2 -> yuvarlansa 2 olurdu) reddi
local frame_dec_seq = make_c_frame(1.0, 2.0, 0.5, 2.2, 3, 12345, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_dec_seq, chan = 0 })
local ev_dec_seq = mission.process_mavlink_queue(mock_time_ms + 100)
report("SENARYO 3k: Ondalikli seq (2.2) yuvarlanmadan reddedildi, eylem yok, zaman degismedi",
    ev_dec_seq == nil and mission.get_last_msg_time_ms() == time_before_invalid)

-- 3l: Ondalıklı Status (param5 = 3.2 -> yuvarlansa 3 olurdu) reddi
local frame_dec_st = make_c_frame(1.0, 2.0, 0.5, 2, 3.2, 12345, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_dec_st, chan = 0 })
local ev_dec_st = mission.process_mavlink_queue(mock_time_ms + 110)
report("SENARYO 3l: Ondalikli status (3.2) yuvarlanmadan reddedildi, eylem yok, zaman degismedi",
    ev_dec_st == nil and mission.get_last_msg_time_ms() == time_before_invalid)

-- 3m: Ondalıklı Session ID (aktif session=12345 iken param6 = 12345.2 -> yuvarlansa 12345 olurdu) reddi
local frame_dec_sess = make_c_frame(1.0, 2.0, 0.5, 2, 3, 12345.2, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_dec_sess, chan = 0 })
local ev_dec_sess = mission.process_mavlink_queue(mock_time_ms + 120)
report("SENARYO 3m: Ondalikli session_id (12345.2) yuvarlanmadan reddedildi, eylem yok, zaman degismedi",
    ev_dec_sess == nil and mission.get_last_msg_time_ms() == time_before_invalid)

-- ============================================================================
-- SENARYO 4: Canlılık Paketi (STATUS_LIVELINESS = 0xAA) ve seq=0 Kabulü
-- ============================================================================
print("\n[SENARYO 4] Canlılık Paketi ve seq=0 Kabulü...")
mock_time_ms = 4500
-- Doğru oturumdan canlılık paketi (seq = 0, session_id = 12345)
local frame_live = make_c_frame(1.0, 0.0, 0.0, 0, 0xAA, 12345, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_live, chan = 0 })
local ev_live = mission.process_mavlink_queue(mock_time_ms)
local s4a_ok = (ev_live == nil) and (mission.get_last_msg_time_ms() == 4500)
report("SENARYO 4a: Canlilik paketi seq=0 ile kabul edildi, last_msg_time guncellendi", s4a_ok)

-- Yanlış oturumdan canlılık paketi (session_id = 99999) -> reddedilmeli, last_msg_time DEĞİŞMEMELİ
mock_time_ms = 4600
local frame_live_wrong = make_c_frame(1.0, 0.0, 0.0, 0, 0xAA, 99999, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_live_wrong, chan = 0 })
local ev_live_wrong = mission.process_mavlink_queue(mock_time_ms)
local s4b_ok = (ev_live_wrong == nil) and (mission.get_last_msg_time_ms() == 4500)
report("SENARYO 4b: Yanlis oturumdan canlilik reddedildi, zaman degismedi", s4b_ok)

-- ============================================================================
-- SENARYO 5: GOTO Tekrarı (Deduplication) ve İçerik Çakışması Reddi
-- ============================================================================
print("\n[SENARYO 5] GOTO Tekrarı ve İçerik Çakışması...")
mock_time_ms = 5000
local tx_prev_g1 = #mock_tx_packets
-- İlk GOTO_OBSERVATION (status = 3, seq = 2, x = 12.0, y = 18.0, conf = 0.9, sess = 12345)
local frame_goto = make_c_frame(12.0, 18.0, 0.9, 2, 3, 12345, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_goto, chan = 0 })
local ev_goto = mission.process_mavlink_queue(mock_time_ms)
local ack_goto = get_new_tx_ack(tx_prev_g1)

local s5a_ok = (ev_goto ~= nil) and
               (ev_goto.status == 3) and
               (ev_goto.seq == 2) and
               (math.abs(ev_goto.x - 12.0) < 0.001) and
               (math.abs(ev_goto.y - 18.0) < 0.001) and
               (ack_goto ~= nil and ack_goto.acked_type == 3 and ack_goto.result == 0 and ack_goto.acked_seq == 2 and ack_goto.acked_sess == 12345 and ack_goto.status_ack == 0xAB)
report("SENARYO 5a: GOTO_OBSERVATION ilk kez alindi, eylem uretildi ve ACK(0) verildi", s5a_ok)

-- Birebir aynı tekrar paketi (retransmission)
mock_time_ms = 5100
local tx_prev_dup = #mock_tx_packets
table.insert(mock_rx_queue, { raw = frame_goto, chan = 0 })
local ev_goto_dup = mission.process_mavlink_queue(mock_time_ms)
local ack_goto_dup = get_new_tx_ack(tx_prev_dup)

local s5b_ok = (ev_goto_dup == nil) and
               (ack_goto_dup ~= nil and ack_goto_dup.acked_type == 3 and ack_goto_dup.result == 0 and ack_goto_dup.acked_seq == 2 and ack_goto_dup.acked_sess == 12345 and ack_goto_dup.status_ack == 0xAB) and
               (mission.get_last_msg_time_ms() == 5100)
report("SENARYO 5b: Birebir ayni tekrar paketi ACK(0) aldi, YENIDEN EYLEM URETILMEDI", s5b_ok)

-- Aynı seq (2) fakat farklı koordinat (x = 99.0) ile çakışan içerik
mock_time_ms = 5200
local tx_prev_cnf = #mock_tx_packets
local frame_conflict = make_c_frame(99.0, 18.0, 0.9, 2, 3, 12345, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_conflict, chan = 0 })
local ev_conflict = mission.process_mavlink_queue(mock_time_ms)
local ack_conflict = get_new_tx_ack(tx_prev_cnf)

local s5c_ok = (ev_conflict == nil) and
               (ack_conflict ~= nil and ack_conflict.acked_type == 3 and ack_conflict.result == 1 and ack_conflict.acked_seq == 2 and ack_conflict.acked_sess == 12345 and ack_conflict.status_ack == 0xAB) and
               (mission.get_last_msg_time_ms() == 5100) -- zaman damgası güncellenmedi!
report("SENARYO 5c: Ayni seq icin farkli icerik REDDEDILDI (ACK=1), zaman damgasi degismedi", s5c_ok)

-- ============================================================================
-- SENARYO 6: Yeni Oturumda Sıra Sıfırlama vs Aynı Oturumun Tekrar İsteği
-- ============================================================================
print("\n[SENARYO 6] Yeni Oturumda Sıra Sıfırlama vs Aynı Oturumun Tekrar İsteği...")
mock_time_ms = 6000
local tx_prev_6a = #mock_tx_packets
-- 6a: Aynı oturumun (12345) tekrar SESSION_START isteği
local frame_same_sess = make_c_frame(0.0, 0.0, 0.0, 1, 0xAC, 12345, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_same_sess, chan = 0 })
mission.process_mavlink_queue(mock_time_ms)
local ack_same_sess = get_new_tx_ack(tx_prev_6a)

local s6a_ok = (ack_same_sess ~= nil and ack_same_sess.acked_type == 0xAC and ack_same_sess.result == 0 and ack_same_sess.acked_seq == 1 and ack_same_sess.acked_sess == 12345 and ack_same_sess.status_ack == 0xAB) and
               (mission.get_accepted_messages()[2] ~= nil) and -- kayıtlı mesajlar korunur!
               (mission.get_last_processed_seq() == 2)
report("SENARYO 6a: Ayni oturum tekrarinda ACK(0) verildi, sira ve kayitlar sifirlanmadi", s6a_ok)

-- 6b: Geçerli YENİ oturum (session_id = 67890, seq = 1)
mock_time_ms = 7000
local tx_prev_6b = #mock_tx_packets
local frame_new_sess = make_c_frame(0.0, 0.0, 0.0, 1, 0xAC, 67890, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_new_sess, chan = 0 })
mission.process_mavlink_queue(mock_time_ms)
local ack_new_sess = get_new_tx_ack(tx_prev_6b)

local accepted = mission.get_accepted_messages()
local count_accepted = 0
for _ in pairs(accepted) do count_accepted = count_accepted + 1 end

local s6b_ok = (ack_new_sess ~= nil and ack_new_sess.acked_type == 0xAC and ack_new_sess.result == 0 and ack_new_sess.acked_seq == 1 and ack_new_sess.acked_sess == 67890 and ack_new_sess.status_ack == 0xAB) and
               (mission.get_active_session_id() == 67890) and
               (mission.get_retired_sessions()[12345] == true) and
               (count_accepted == 0) and
               (mission.get_last_processed_seq() == -1) and
               (mission.get_SEQ_ESIGI() == -1)
report("SENARYO 6b: Yeni oturumda eski oturum kapatildi, SEQ_ESIGI ve last_processed_seq (-1) sifirlandi", s6b_ok)

-- ============================================================================
-- SENARYO 7: Kapatılmış / Emekliye Ayrılmış Oturum Reddi
-- ============================================================================
print("\n[SENARYO 7] Kapatılmış Oturum Reddi...")
mock_time_ms = 8000
local tx_prev_7 = #mock_tx_packets
-- Kapatılmış oturum (12345) için gecikmiş SESSION_START
local frame_stale_sess = make_c_frame(0.0, 0.0, 0.0, 1, 0xAC, 12345, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_stale_sess, chan = 0 })
mission.process_mavlink_queue(mock_time_ms)
local ack_stale_sess = get_new_tx_ack(tx_prev_7)

local s7_ok = (ack_stale_sess ~= nil and ack_stale_sess.acked_type == 0xAC and ack_stale_sess.result == 1 and ack_stale_sess.acked_seq == 1 and ack_stale_sess.acked_sess == 12345 and ack_stale_sess.status_ack == 0xAB) and
              (mission.get_active_session_id() == 67890)
report("SENARYO 7: Kapatilmis eski oturum istegi REDDEDILDI (ACK=1), aktif oturum degismedi", s7_ok)

-- ============================================================================
-- SENARYO 8: Aynı Kuyrukta Oturum Değişince Eski Eylemin Temizlenmesi
-- ============================================================================
print("\n[SENARYO 8] Aynı Kuyrukta Oturum Değişince Eski Eylemin Temizlenmesi...")
mock_time_ms = 9000
-- Kuyruğa 2 mesaj ekle:
-- 1: Aktif oturumdan (67890) GOTO_OBSERVATION (seq = 2)
-- 2: Yepyeni oturum (88888) SESSION_START (seq = 1)
local frame_old_action = make_c_frame(5.0, 5.0, 0.9, 2, 3, 67890, 1.0, 31010, 1, 1, 0, 1, 191, 1)
local frame_queue_new_sess = make_c_frame(0.0, 0.0, 0.0, 1, 0xAC, 88888, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = frame_old_action, chan = 0 })
table.insert(mock_rx_queue, { raw = frame_queue_new_sess, chan = 0 })

local ev_queue_result = mission.process_mavlink_queue(mock_time_ms)

local s8_ok = (ev_queue_result == nil) and
              (mission.get_active_session_id() == 88888) and
              (mission.get_retired_sessions()[67890] == true)
report("SENARYO 8: Ayni kuyrukta yeni oturum acilinca eski oturuma ait eylem FSM'e donmedi (nil)", s8_ok)

-- ============================================================================
-- SENARYO 9: Canlılık Sürerken 90 Saniyelik Görev Zaman Aşımı Failsafe
-- ============================================================================
print("\n[SENARYO 9] Canlılık Sürerken 90s Görev Zaman Aşımı (TIMEOUT_HEDEF_BEKLE_MS)...")
-- Mock uçuş bağlamını (BASLANGIC_KONUMU, ILERI_YAW, hedef vektörleri) hazırla
local upval_i = 1
while true do
    local name, _ = debug.getupvalue(mission.update, upval_i)
    if not name then break end
    if name == "BASLANGIC_KONUMU" then
        debug.setupvalue(mission.update, upval_i, { kuzey = 0.0, dogu = 0.0, z = 0.0 })
    elseif name == "ILERI_YAW" then
        debug.setupvalue(mission.update, upval_i, 0.0)
    elseif name == "kalkis_hedef_kuzey" then
        debug.setupvalue(mission.update, upval_i, 40.0)
    elseif name == "kalkis_hedef_dogu" then
        debug.setupvalue(mission.update, upval_i, 0.0)
    elseif name == "kalkis_hedef_z" then
        debug.setupvalue(mission.update, upval_i, -33.0)
    end
    upval_i = upval_i + 1
end

mock_vehicle_mode = 4 -- COPTER_MODE_GUIDED
mock_is_armed = true
local t_entry = 10000
mock_time_ms = t_entry
mission.change_state(4) -- STATE_HEDEF_BEKLE
mission.set_state_entry_time_ms(t_entry)

local active_sess = mission.get_active_session_id()
local premature_donus = false

-- Zamanı 10.0s'den 99.0s'ye (89 saniye boyunca) 2 Hz canlılık ve update() çağrılarıyla ilerlet
for t = t_entry + 500, t_entry + 89000, 500 do
    mock_time_ms = t
    local live_pkt = make_c_frame(1.0, 0.0, 0.0, 0, 0xAA, active_sess, 1.0, 31010, 1, 1, 0, 1, 191, 1)
    table.insert(mock_rx_queue, { raw = live_pkt, chan = 0 })
    mission.update()
    if mission.get_current_state() ~= 4 then
        premature_donus = true
        break
    end
end

local s9_89s_ok = (not premature_donus) and
                  (mission.get_current_state() == 4) and
                  (mission.get_last_msg_time_ms() == (t_entry + 89000))
report("SENARYO 9a: 89. saniyede 2 Hz canlilik ile baglanti taze kaldi, donus baslamadi (State=4)", s9_89s_ok)

-- 90,05 saniyede update öncesinde geçerli canlılık paketini kuyruğa ekle
local t_final = t_entry + 90050
mock_time_ms = t_final
local live_final_pkt = make_c_frame(1.0, 0.0, 0.0, 0, 0xAA, active_sess, 1.0, 31010, 1, 1, 0, 1, 191, 1)
table.insert(mock_rx_queue, { raw = live_final_pkt, chan = 0 })

mission.update()
local state_after_timeout = mission.get_current_state()
local last_msg_after_timeout = mission.get_last_msg_time_ms()

local s9_90s_ok = (state_after_timeout == 7) and (last_msg_after_timeout == t_final)
report("SENARYO 9b: 90,05s'de canlilik islendi (zaman guncellendi) ve gorev zaman asimiyla DONUS (State=7) tetiklendi", s9_90s_ok,
    string.format("State=%d, last_msg_time=%d, Test suresi=%.2fs", state_after_timeout, last_msg_after_timeout, (t_final - t_entry) / 1000.0))

-- ============================================================================
-- SENARYO 10: send_app_ack Fonksiyonunun send_chan Başarısızlığını Yansıtması
-- ============================================================================
print("\n[SENARYO 10] send_app_ack send_chan Başarısızlığını Doğru Yansıtması...")
mock_send_chan_result = false
local tx_fail_res = mission.send_app_ack(0xAC, 0, 1, 88888, 0)

mock_send_chan_result = true
local tx_success_res = mission.send_app_ack(0xAC, 0, 1, 88888, 0)

local s10_ok = (tx_fail_res == false) and (tx_success_res == true)
report("SENARYO 10: send_app_ack basarisiz gonderimi (false) ve basarili gonderimi (true) dogru yansitti", s10_ok)

-- ============================================================================
-- ÖZET VE ÇIKIŞ KODU
-- ============================================================================
print("\n============================================================================")
print(string.format("TEST SONUÇLARI ÖZETİ: %d Toplam | %d BAŞARILI | %d BAŞARISIZ",
    total_tests, passed_tests, failed_tests))
print("============================================================================")

if failed_tests > 0 then
    print(string.format("HATA: %d test basarisiz oldu! Cikis kodu: 1", failed_tests))
    os.exit(1)
else
    print("TEBRİKLER: Tum izole Lua alici testleri basariyla tamamlandi! Cikis kodu: 0")
    os.exit(0)
end
