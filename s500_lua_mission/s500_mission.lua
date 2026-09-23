--[[
    ============================================================================
    S500 Otonom Görev Durum Makinesi (s500_mission.lua)
    ============================================================================
    ArduPilot AP_Scripting Motoru (Cube Orange & SITL)
    ArduCopter V4.7.1 API Uyumlu
    
    FİZİKSEL KURULUM PRENSİBİ:
    - Cube Orange üzerindeki yön oku drone sahaya konurken alana (orta sahaya)
      doğru çevrilir.
    - "İleri yön", drone henüz yerdeyken okunan ilk yaw açısı (ILERI_YAW) üzerinden
      trigonometrik olarak (cos/sin) hesaplanır. Sabit pusula açısı kullanılmaz;
      kalkış hangi penaltı noktasından olursa olsun tutarlı çalışır.
    - ILERI_YAW yalnızca HAZIRLIK durumunda BİR KEZ kilitlenir; rüzgar veya dönüş
      manevralarından sonra asla güncellenmez.

    GÖREV AKIŞI (DURUM MAKİNESİ):
    1. BEKLEME: RC anahtarı veya GUIDED moda geçiş tetiklemesi beklenir.
    2. HAZIRLIK:
       - ahrs:get_relative_position_NED_origin() ile 5 örnek alınıp ortalanır (BASLANGIC_KONUMU).
       - ILERI_YAW yerdeyken bir kez okunup dondurulur.
       - param:set ile WP_SPD=1.0 m/s ayarlanır, param:get ile ±0.05 toleransla doğrulanır.
       - Başarısızsa iptal edilip BEKLEME'ye dönülür; başarılıysa GUIDED + Arm edilip KALKIŞ'a geçilir.
    3. KALKIŞ:
       - Birleşik 3D hareket: 40m ileri (ILERI_YAW doğrultusunda) + 33m irtifa.
       - Yatay mesafe < 2m VE |irtifa - 33m| < 0.5m (31-35m bandı) içinde 2 saniye
         kararlılık beklenip HEDEF_BEKLE'ye geçilir.
    4. HEDEF_BEKLE:
       - Girişte SEQ_ESIGI = en son işlenmiş seq kaydedilir (state-entry marker).
       - MAVLink COMMAND_LONG (31010 / MAV_CMD_USER_1) dinlenir (bloklu ACK).
       - Sadece seq > SEQ_ESIGI olan taze mesajlar işlenir.
       - status == 3 (GOTO_OBSERVATION) -> HEDEF_KONUM kaydedilir, HEDEFE_GİT'e geçilir.
       - status == 4 (ROUTE_READY) -> Doğrudan DÖNÜŞ'e geçilir (kısa yol).
       - status == 0, 1, 2, 5 -> Bilgi olarak loglanır, beklenir.
       - 5s veri gelmezse "link koptu" loglanır.
       - 90s içinde 3 veya 4 gelmezse failsafe olarak doğrudan DÖNÜŞ'e geçilir.
    5. HEDEFE_GİT:
       - HEDEF_KONUM'a (x, y) 33m sabit irtifada uçulur.
       - Yatay mesafe < 1m olunca 2 saniye kararlılık beklenir, HEDEF_KONUMUNDA_BEKLE'ye geçilir.
    6. HEDEF_KONUMUNDA_BEKLE:
       - Girişte SEQ_ESIGI güncellenir.
       - status == 4 -> DÖNÜŞ'e geçilir.
       - status == 3 -> HEDEF_KONUM güncellenir, HEDEFE_GİT'e dönülür.
       - status == 0, 1, 2, 5 -> Bilgi olarak loglanır, beklenir.
       - 60s içinde status 4 gelmezse failsafe DÖNÜŞ'e geçilir.
    7. DÖNÜŞ:
       - HAZIRLIK'ta kaydedilen gerçek BASLANGIC_KONUMU'na (asla (0,0) değil) 33m irtifada uçulur.
       - Yatay mesafe < 1m olunca 2 saniye kararlılık beklenir, İNİŞ'e geçilir.
    8. İNİŞ:
       - LAND moduna geçilir (Mode 9).
       - Disarm teyidi alınınca "GÖREV TAMAMLANDI" loglanır.
    ============================================================================
--]]

-- MAVLink mesaj kütüphanesini opsiyonel olarak yükle (SD kartta mevcutsa)
local has_mavlink_msgs, mavlink_msgs = pcall(require, "MAVLink/mavlink_msgs")
local msg_map = {}
if has_mavlink_msgs and mavlink_msgs then
    pcall(function()
        local cmd_long_id = mavlink_msgs.get_msgid("COMMAND_LONG")
        msg_map[cmd_long_id] = "COMMAND_LONG"
    end)
end

-- ============================================================================
-- SABİTLER VE YAPILANDIRMA
-- ============================================================================

-- MAV_SEVERITY Sabitleri
local MAV_SEVERITY = {
    EMERGENCY = 0,
    ALERT     = 1,
    CRITICAL  = 2,
    ERROR     = 3,
    WARNING   = 4,
    NOTICE    = 5,
    INFO      = 6,
    DEBUG     = 7
}

-- Copter Uçuş Modları (ArduCopter V4.7.1)
local COPTER_MODE_GUIDED = 4
local COPTER_MODE_LAND   = 9

-- MAVLink Protokol Sabitleri
local MAVLINK_MSG_ID_COMMAND_LONG = 76
local MAV_CMD_USER_1              = 31010 -- Companion bilgisayar (Jetson) komut kimliği

-- Görev Durumları (State Machine)
local STATE_BEKLEME               = 1
local STATE_HAZIRLIK              = 2
local STATE_KALKIS                = 3
local STATE_HEDEF_BEKLE           = 4
local STATE_HEDEFE_GIT            = 5
local STATE_HEDEF_KONUMUNDA_BEKLE = 6
local STATE_DONUS                 = 7
local STATE_INIS                  = 8
local STATE_TAMAMLANDI            = 9
local STATE_PILOT_MUDAHALESI      = 10
local STATE_ROUTE_EXECUTE         = 11

local STATE_NAMES = {
    [STATE_BEKLEME]               = "BEKLEME",
    [STATE_HAZIRLIK]              = "HAZIRLIK",
    [STATE_KALKIS]                = "KALKIS",
    [STATE_HEDEF_BEKLE]           = "HEDEF_BEKLE",
    [STATE_HEDEFE_GIT]            = "HEDEFE_GIT",
    [STATE_HEDEF_KONUMUNDA_BEKLE] = "HEDEF_KONUMUNDA_BEKLE",
    [STATE_DONUS]                 = "DONUS",
    [STATE_INIS]                  = "INIS",
    [STATE_TAMAMLANDI]            = "TAMAMLANDI",
    [STATE_PILOT_MUDAHALESI]      = "PILOT_MUDAHALESI",
    [STATE_ROUTE_EXECUTE]         = "ROUTE_EXECUTE"
}

-- Olay Durum Kodları (Jetson Sözleşmesi - param5)
local STATUS_NO_DETECTION      = 0
local STATUS_START_FOUND       = 1
local STATUS_GOAL_FOUND        = 2
local STATUS_GOTO_OBSERVATION  = 3
local STATUS_ROUTE_READY       = 4
local STATUS_GCS_DELIVERED     = 5

local STATUS_NAMES = {
    [STATUS_NO_DETECTION]     = "NO_DETECTION",
    [STATUS_START_FOUND]      = "START_FOUND",
    [STATUS_GOAL_FOUND]       = "GOAL_FOUND",
    [STATUS_GOTO_OBSERVATION] = "GOTO_OBSERVATION",
    [STATUS_ROUTE_READY]      = "ROUTE_READY",
    [STATUS_GCS_DELIVERED]    = "GCS_DELIVERED"
}

-- MAVLink Jetson Protokol Sabitleri (vision_lua_protocol.md)
local STATUS_LIVELINESS        = 0xAA  -- 170: Jetson canlılık/sağlık durumu
local STATUS_APP_ACK           = 0xAB  -- 171: Lua'dan Jetson'a özel APP_ACK teyidi
local STATUS_SESSION_START     = 0xAC  -- 172: Jetson oturum açma isteği
local PROTOCOL_VERSION         = 1.0   -- Desteklenen protokol sürümü (param7)
local COMPANION_SYSID          = 1     -- Companion bilgisayar System ID
local COMPANION_COMPID         = 191   -- Companion bilgisayar Component ID

-- Uçuş Geometrisi ve Tolerans Yapılandırması
local HEDEF_IRTIFA_M          = 33.0   -- Hedef seyir irtifası (m)
local IRTIFA_ALT_SINIR_M      = 31.0   -- İrtifa kabul bandı alt sınır (m)
local IRTIFA_UST_SINIR_M      = 35.0   -- İrtifa kabul bandı üst sınır (m)
local IRTIFA_HATA_TOLERANS_M  = 0.5    -- İrtifa yaklaşım toleransı (m)

local KALKIS_ILERELEME_M      = 40.0   -- İleri yönlü 3D kalkış mesafesi (m)
local KALKIS_YATAY_TOLERANS_M = 2.0    -- Kalkış yatay varış toleransı (m)
local GOZLEM_YATAY_TOLERANS_M = 1.0    -- Gözlem noktası yatay toleransı (m)
local DONUS_YATAY_TOLERANS_M  = 1.0    -- Dönüş kalkış noktası yatay toleransı (m)

local KARARLILIK_SURESI_MS    = 2000   -- 2.0 saniye kesintisiz kararlılık süresi
local LINK_KOPMA_TIMEOUT_MS   = 5000   -- 5.0 saniye mesaj yoksa link koptu uyarısı
local TIMEOUT_HEDEF_BEKLE_MS  = 90000  -- 90 saniye zaman aşımı (HEDEF_BEKLE failsafe)
local TIMEOUT_KONUM_BEKLE_MS  = 60000  -- 60 saniye zaman aşımı (HEDEF_KONUMUNDA_BEKLE failsafe)
local TIMEOUT_GENEL_HAREKET_MS= 120000 -- 120 saniye intikal zaman aşımı

local UPDATE_RATE_MS          = 100    -- Ana döngü frekansı (10 Hz)

-- ============================================================================
-- DURUM DEĞİŞKENLERİ
-- ============================================================================

local current_state         = STATE_BEKLEME
local state_entry_time_ms   = 0
local mission_index         = 1

-- Konum ve Yön Kayıtları
local BASLANGIC_KONUMU      = nil      -- { kuzey = float, dogu = float, z = float }
local ILERI_YAW             = nil      -- Yerde bir kez okunan ilk yaw açısı (radyan)
local HEDEF_KONUM           = nil      -- { x = float, y = float } (NED Kuzey/Doğu)

-- Hazırlık Aşaması Değişkenleri
local hazirlik_samples      = {}       -- EKF relative_position_NED_origin örnekleri
local arm_requested         = false
local arm_request_time_ms   = 0

-- Kalkış Değişkenleri
local kalkis_started        = false
local kalkis_3d_steered     = false
local kalkis_hedef_kuzey    = 0.0
local kalkis_hedef_dogu     = 0.0
local kalkis_hedef_z        = 0.0

-- Kararlılık Sayacı
local stability_start_ms    = nil

-- MAVLink Mesaj ve Tazelik Takibi
local SEQ_ESIGI             = -1       -- Durum girişinde güncellenen tazelik eşiği
local last_processed_seq    = -1       -- En son işlenmiş mesajın seq numarası
local last_msg_time_ms      = nil      -- Son MAV_CMD_USER_1 mesaj zaman damgası
local link_lost_logged      = false    -- Link koptu mesajının tekrarını önleme bayrağı
local active_session_id     = nil      -- Jetson oturum açma (SESSION_START) ile doğrulanan kimlik
local retired_sessions      = {}       -- Kapanmis/eski oturum ID'lerinin kaydi
local accepted_messages     = {}       -- Aktif oturumda kabul edilen mesajlar: [seq] = {status, x, y, conf}
local start_record          = nil      -- STATUS_START_FOUND kaydi: {x, y, conf, seq}
local goal_record           = nil      -- STATUS_GOAL_FOUND kaydi: {x, y, conf, seq}
local MAX_INT_24BIT         = 16777215 -- 2^24 - 1: IEEE-754 float tam sayi siniri

-- ============================================================================
-- YARDIMCI FONKSİYONLAR
-- ============================================================================

-- Zamanı milisaniye cinsinden sayı (float) olarak alma
local function get_time_ms()
    local t = millis()
    if type(t) == "userdata" and t.tofloat then
        return t:tofloat()
    end
    return t
end

-- GCS Bilgilendirme Mesajları
local function log_info(msg)
    if gcs and gcs.send_text then
        gcs:send_text(MAV_SEVERITY.INFO, "[S500 LUA] " .. msg)
    end
end

local function log_warn(msg)
    if gcs and gcs.send_text then
        gcs:send_text(MAV_SEVERITY.WARNING, "[S500 LUA] " .. msg)
    end
end

local function log_error(msg)
    if gcs and gcs.send_text then
        gcs:send_text(MAV_SEVERITY.ERROR, "[S500 LUA] " .. msg)
    end
end

-- Vector3f Nesnesi Oluşturma ve Bileşenlerini Ayarlama
local function make_vector3f(x, y, z)
    local v = Vector3f()
    v:x(x)
    v:y(y)
    v:z(z)
    return v
end

-- Durum Geçiş Yöneticisi
local function change_state(new_state)
    local old_name = STATE_NAMES[current_state] or tostring(current_state)
    local new_name = STATE_NAMES[new_state] or tostring(new_state)
    
    current_state = new_state
    state_entry_time_ms = get_time_ms()
    stability_start_ms = nil

    -- State-entry marker: HEDEF_BEKLE ve HEDEF_KONUMUNDA_BEKLE girişinde tazelik eşiği kilitlenir
    if new_state == STATE_HEDEF_BEKLE or new_state == STATE_HEDEF_KONUMUNDA_BEKLE then
        SEQ_ESIGI = last_processed_seq
        link_lost_logged = false
        log_info(string.format("Durum Gecisi: %s -> %s (Yeni SEQ_ESIGI=%d kaydedildi)",
            old_name, new_name, SEQ_ESIGI))
    else
        log_info(string.format("Durum Gecisi: %s -> %s", old_name, new_name))
    end
end

-- ============================================================================
-- MAVLINK GELEN MESAJ AYRIŞTIRMA (COMMAND_LONG / MAV_CMD_USER_1)
-- ============================================================================

-- Ham sayı ve tam sayı geçerlilik denetleyicileri
local function is_valid_raw_number(val)
    return type(val) == "number" and val == val and val ~= math.huge and val ~= -math.huge
end

local function is_valid_raw_int(val, min_val, max_val)
    if not is_valid_raw_number(val) then
        return false
    end
    if val ~= math.floor(val) then
        return false
    end
    if val < min_val or val > max_val then
        return false
    end
    return true
end

-- Koordinat ve güvenilirlik denetleyicileri
local function is_valid_coordinates(x, y)
    return is_valid_raw_number(x) and is_valid_raw_number(y)
end

local function is_valid_confidence(conf, status)
    if not is_valid_raw_number(conf) or conf < 0.0 or conf > 1.0 then
        return false
    end
    -- Hedef tespit olaylarında (status 1..4) adaptör strictly conf > 0.0 şartı arar
    if (status >= 1 and status <= 4) and conf <= 0.0 then
        return false
    end
    return true
end

-- MAVLink COMMAND_LONG paketini ayrıştırır (Modül varsa birincil olarak modülü, yoksa yedek binary unpack kullanır)
local function parse_command_long(raw_bytes)
    if not raw_bytes or #raw_bytes < 30 then
        return nil
    end

    -- 1. ZORUNLU BİRİNCİL YOL: Resmi ArduPilot MAVLink Lua modülü mevcutsa kullan
    if has_mavlink_msgs and mavlink_msgs then
        local ok, parsed = pcall(mavlink_msgs.decode, raw_bytes, msg_map)
        if ok and parsed and parsed.command then
            return {
                command          = parsed.command,
                param1           = parsed.param1,
                param2           = parsed.param2,
                param3           = parsed.param3,
                param4           = parsed.param4,
                param5           = parsed.param5,
                param6           = parsed.param6,
                param7           = parsed.param7,
                target_system    = parsed.target_system,
                target_component = parsed.target_component,
                confirmation     = parsed.confirmation,
                src_sys          = parsed.sysid,
                src_comp         = parsed.compid
            }
        end
    end

    -- 2. YEDEK YOL: Modül yoksa doğrudan binary unpack
    -- Durum A: ArduPilot C++ 'mavlink_message_t' yapısı (Header + Payload, offset 13'te payload başlar)
    if #raw_bytes >= 45 then
        local ok_magic, magic = pcall(string.unpack, "<B", raw_bytes, 3)
        if ok_magic and (magic == 0xFD or magic == 0xFE) then
            local ok_id, msgid = pcall(string.unpack, "<I3", raw_bytes, 10)
            if ok_id and msgid == MAVLINK_MSG_ID_COMMAND_LONG then
                local ok_src, src_seq, src_sys, src_comp = pcall(string.unpack, "<BBB", raw_bytes, 7)
                local ok_payload, p1, p2, p3, p4, p5, p6, p7, cmd, tgt_sys, tgt_comp, conf = pcall(string.unpack, "<fffffffI2BBB", raw_bytes, 13)
                if ok_payload and ok_src then
                    return {
                        command          = cmd,
                        param1           = p1,
                        param2           = p2,
                        param3           = p3,
                        param4           = p4,
                        param5           = p5,
                        param6           = p6,
                        param7           = p7,
                        target_system    = tgt_sys,
                        target_component = tgt_comp,
                        confirmation     = conf,
                        src_sys          = src_sys,
                        src_comp         = src_comp
                    }
                end
            end
        end
    end

    -- Durum B: Saf Payload (Header yok, kaynak ayrıştırılamaz -> src_sys ve src_comp nil döner)
    if #raw_bytes >= 33 then
        local ok_raw, p1, p2, p3, p4, p5, p6, p7, cmd, tgt_sys, tgt_comp, conf = pcall(string.unpack, "<fffffffI2BBB", raw_bytes, 1)
        if ok_raw and cmd == MAV_CMD_USER_1 then
            return {
                command          = cmd,
                param1           = p1,
                param2           = p2,
                param3           = p3,
                param4           = p4,
                param5           = p5,
                param6           = p6,
                param7           = p7,
                target_system    = tgt_sys,
                target_component = tgt_comp,
                confirmation     = conf,
                src_sys          = nil,
                src_comp         = nil
            }
        end
    end

    return nil
end

-- Jetson'a APP_ACK (STATUS_APP_ACK = 0xAB) yanıtı gönderme
local function send_app_ack(acked_type, result, acked_seq, acked_sess, chan)
    chan = chan or 0
    local target_sys = COMPANION_SYSID
    local target_comp = COMPANION_COMPID

    local msg_table = {
        param1           = tonumber(acked_type) or 0.0,
        param2           = tonumber(result) or 0.0,
        param3           = 0.0,
        param4           = tonumber(acked_seq) or 0.0,
        param5           = tonumber(STATUS_APP_ACK) or 171.0,
        param6           = tonumber(acked_sess) or 0.0,
        param7           = tonumber(PROTOCOL_VERSION) or 1.0,
        command          = MAV_CMD_USER_1,
        target_system    = target_sys,
        target_component = target_comp,
        confirmation     = 0
    }

    if mavlink and mavlink.send_chan then
        if has_mavlink_msgs and mavlink_msgs and mavlink_msgs.encode then
            local ok, id, payload = pcall(mavlink_msgs.encode, "COMMAND_LONG", msg_table)
            if ok and id and payload then
                local send_ok = mavlink:send_chan(chan, id, payload)
                return (send_ok == true)
            end
        end
        local ok, payload = pcall(string.pack, "<fffffffI2BBB",
            msg_table.param1, msg_table.param2, msg_table.param3,
            msg_table.param4, msg_table.param5, msg_table.param6, msg_table.param7,
            msg_table.command, msg_table.target_system, msg_table.target_component, msg_table.confirmation
        )
        if ok and payload then
            local send_ok = mavlink:send_chan(chan, MAVLINK_MSG_ID_COMMAND_LONG, payload)
            return (send_ok == true)
        end
    end
    return false
end

-- Kuyruktaki tüm MAVLink mesajlarını tüketir, SEQ_ESIGI üzerindeki en son geçerli eylemi döndürür
local function process_mavlink_queue(current_time_ms)
    local latest_action_event = nil

    if not mavlink or not mavlink.receive_chan then
        return nil
    end

    while true do
        local raw_msg, chan = mavlink:receive_chan()
        if not raw_msg then
            break
        end

        local cmd = parse_command_long(raw_msg)
        if cmd and cmd.command == MAV_CMD_USER_1 then
            -- 1. Kaynak Adres Doğrulaması (Eksikse veya yetkisizse mesaj reddedilir)
            if cmd.src_sys == nil or cmd.src_comp == nil then
                log_warn("Kaynak system veya component ayrilamadi! Mesaj reddedildi.")
                goto continue_loop
            end
            if cmd.src_sys ~= COMPANION_SYSID or cmd.src_comp ~= COMPANION_COMPID then
                log_warn(string.format("Yetkisiz kaynak adresi (sys=%d, comp=%d)! Mesaj reddedildi.", cmd.src_sys, cmd.src_comp))
                goto continue_loop
            end

            -- 2. Hedef Adres Doğrulaması (Eksikse veya geçersizse mesaj reddedilir)
            if cmd.target_system == nil or cmd.target_component == nil then
                log_warn("target_system veya target_component ayrilamadi! Mesaj reddedildi.")
                goto continue_loop
            end
            if cmd.target_system ~= 1 or (cmd.target_component ~= 1 and cmd.target_component ~= 0) then
                log_warn(string.format("Gecersiz hedef adresi (sys=%s, comp=%s)! Mesaj reddedildi.",
                    tostring(cmd.target_system), tostring(cmd.target_component)))
                goto continue_loop
            end

            -- 3. Protokol Sürümü Doğrulaması (Yuvarlama yok, doğrudan PROTOCOL_VERSION karşılaştırması)
            if not is_valid_raw_number(cmd.param7) or cmd.param7 ~= PROTOCOL_VERSION then
                log_warn(string.format("Desteklenmeyen protokol surumu: %s! Mesaj reddedildi.", tostring(cmd.param7)))
                goto continue_loop
            end

            -- 4. Status Alanı Ham Doğrulaması (Sayı, sonlu, tam sayı, protokol aralığı: 0..5 veya 0xAA veya 0xAC)
            local status = cmd.param5
            local is_status_valid = is_valid_raw_int(status, 0, 5) or status == STATUS_LIVELINESS or status == STATUS_SESSION_START
            if not is_status_valid then
                log_warn(string.format("Bilinmeyen veya gecersiz status (%s) alindi! Reddedildi.", tostring(status)))
                goto continue_loop
            end

            -- 5. Session ID Ham Doğrulaması (Sayı, sonlu, tam sayı, 1 <= session_id <= 16777215)
            local sess_id = cmd.param6
            if not is_valid_raw_int(sess_id, 1, MAX_INT_24BIT) then
                log_warn(string.format("Gecersiz session_id (%s)! Mesaj reddedildi.", tostring(sess_id)))
                if status == STATUS_SESSION_START then
                    send_app_ack(status, 1, cmd.param4 or 0, 0, chan)
                end
                goto continue_loop
            end

            -- 6. Sequence Numarası Ham Doğrulaması (Sayı, sonlu, tam sayı, liveliness için >= 0, diğerleri için 1..16777215)
            local seq = cmd.param4
            local min_seq = (status == STATUS_LIVELINESS) and 0 or 1
            if not is_valid_raw_int(seq, min_seq, MAX_INT_24BIT) then
                log_warn(string.format("Gecersiz seq (%s) alindi! Mesaj reddedildi.", tostring(seq)))
                if status ~= STATUS_LIVELINESS then
                    send_app_ack(status, 1, 0, sess_id, chan)
                end
                goto continue_loop
            end

            -- 7. Canlılık Mesajı (0xAA: STATUS_LIVELINESS)
            if status == STATUS_LIVELINESS then
                if active_session_id and sess_id == active_session_id then
                    last_msg_time_ms = current_time_ms
                    if link_lost_logged then
                        link_lost_logged = false
                        log_info("Jetson ile link yeniden kuruldu.")
                    end
                else
                    log_warn(string.format("Bilinmeyen veya uyumsuz oturumdan canlilik paketi (Sess: %d, Aktif: %s)!",
                        sess_id, tostring(active_session_id)))
                end
                goto continue_loop
            end

            -- 8. Oturum Açma Mesajı (0xAC: STATUS_SESSION_START)
            if status == STATUS_SESSION_START then
                if sess_id == active_session_id then
                    -- Aynı oturumun tekrar isteği: sayaçları sıfırlamadan ACK'yi tekrarla
                    log_info(string.format("Ayni oturum (ID: %d) icin tekrar SESSION_START alindi, ACK gonderildi.", sess_id))
                    last_msg_time_ms = current_time_ms
                    if link_lost_logged then
                        link_lost_logged = false
                        log_info("Jetson ile link yeniden kuruldu.")
                    end
                    send_app_ack(status, 0, seq, sess_id, chan)
                elseif retired_sessions[sess_id] then
                    -- Gecikmiş / kapanmış eski oturum isteği: REDDEDİLDİ (last_msg_time_ms güncellenmez)
                    log_warn(string.format("Gecikmis/kapanmis eski oturum (ID: %d) istegi alindi! Reddedildi.", sess_id))
                    send_app_ack(status, 1, seq, sess_id, chan)
                else
                    -- Geçerli YENİ oturum!
                    if active_session_id then
                        retired_sessions[active_session_id] = true
                        log_info(string.format("Eski oturum (ID: %d) kapatildi/emekliye ayrildi.", active_session_id))
                    end
                    active_session_id = sess_id
                    accepted_messages = {}
                    start_record = nil
                    goal_record = nil
                    latest_action_event = nil   -- Kuyruktaki eski oturuma ait eylemi temizle!
                    last_processed_seq = -1
                    SEQ_ESIGI = -1              -- Yeni oturumun sıra düzenine uygun birlikte başlatıldı!
                    last_msg_time_ms = current_time_ms
                    if link_lost_logged then
                        link_lost_logged = false
                        log_info("Jetson ile link yeniden kuruldu.")
                    end
                    send_app_ack(status, 0, seq, sess_id, chan)
                    log_info(string.format("Yeni Jetson oturumu basariyla acildi (Session ID: %d, Seq: %d).", sess_id, seq))
                end
                goto continue_loop
            end

            -- 9. Aktif Oturum Kontrolü (Diğer tüm durumlar için aktif oturum zorunludur)
            if not active_session_id or sess_id ~= active_session_id then
                log_warn(string.format("Aktif oturum disinda mesaj (Mesaj Sess: %d, Aktif: %s)! Reddedildi.",
                    sess_id, tostring(active_session_id)))
                send_app_ack(status, 1, seq, sess_id, chan)
                goto continue_loop
            end

            local x          = cmd.param1
            local y          = cmd.param2
            local confidence = cmd.param3

            -- 10. Koordinat ve Güvenilirlik (Confidence) Doğrulaması
            if not is_valid_coordinates(x, y) then
                log_warn(string.format("Gecersiz x/y koordinatlari (x=%s, y=%s)! Mesaj reddedildi.", tostring(x), tostring(y)))
                send_app_ack(status, 1, seq, sess_id, chan)
                goto continue_loop
            end

            if not is_valid_confidence(confidence, status) then
                log_warn(string.format("Gecersiz confidence (%.3f, status=%d) alindi! Mesaj reddedildi.", confidence, status))
                send_app_ack(status, 1, seq, sess_id, chan)
                goto continue_loop
            end

            -- 11. Tekrar Mesaj ve İçerik Doğrulaması (Toleranssız birebir eşitlik)
            if accepted_messages[seq] then
                local prev = accepted_messages[seq]
                if prev.status == status and prev.x == x and prev.y == y and prev.conf == confidence then
                    -- Birebir aynı içerikle tekrar iletimi: Başarılı ACK tekrarla, aksiyon üretme
                    send_app_ack(status, 0, seq, sess_id, chan)
                    last_msg_time_ms = current_time_ms
                    if link_lost_logged then
                        link_lost_logged = false
                        log_info("Jetson ile link yeniden kuruldu.")
                    end
                    log_info(string.format("Tekrar mesaj basariyla teyit edildi (Seq: %d, Status: %d). Yeniden islenmedi.", seq, status))
                else
                    -- Aynı seq ancak farklı içerik: Başarısızlık ACK'si (result=1) gönder (last_msg_time_ms DEĞİŞTİRİLMEZ)
                    send_app_ack(status, 1, seq, sess_id, chan)
                    log_warn(string.format("Ayni seq (%d) icin farkli icerik alindi! Reddedildi.", seq))
                end
                goto continue_loop
            end

            -- 12. Durum Tazelik Eşiği Kontrolü (SEQ_ESIGI)
            if seq <= SEQ_ESIGI then
                log_warn(string.format("Bayat mesaj (Seq: %d <= SEQ_ESIGI: %d). Reddedildi.", seq, SEQ_ESIGI))
                send_app_ack(status, 1, seq, sess_id, chan)
                goto continue_loop
            end

            -- 13. Yeni Geçerli Mesajın Kabul Edilmesi ve İşlenmesi
            accepted_messages[seq] = {
                status = status,
                x      = x,
                y      = y,
                conf   = confidence
            }
            if seq > last_processed_seq then
                last_processed_seq = seq
            end

            -- Mesaj kabul edildi: Zaman damgası ve link durumu güncellenir
            last_msg_time_ms = current_time_ms
            if link_lost_logged then
                link_lost_logged = false
                log_info("Jetson ile link yeniden kuruldu.")
            end

            -- Başarı teyidi (APP_ACK result=0)
            send_app_ack(status, 0, seq, sess_id, chan)

            local status_name = STATUS_NAMES[status] or tostring(status)

            -- Bilgi Amaçlı Olaylar (0, 1, 2, 5): Yalnızca loglanır, durum geçişi tetiklemez
            if status == STATUS_NO_DETECTION then
                log_info(string.format("Jetson Bildirimi: %s (Seq: %d, Conf: %.2f)", status_name, seq, confidence))
            elseif status == STATUS_START_FOUND then
                start_record = { x = x, y = y, conf = confidence, seq = seq }
                log_info(string.format("Jetson Bildirimi: %s (Seq: %d, Hedef: [%.1f, %.1f], Conf: %.2f) - Kaydedildi.",
                    status_name, seq, x, y, confidence))
            elseif status == STATUS_GOAL_FOUND then
                goal_record = { x = x, y = y, conf = confidence, seq = seq }
                log_info(string.format("Jetson Bildirimi: %s (Seq: %d, Hedef: [%.1f, %.1f], Conf: %.2f) - Kaydedildi.",
                    status_name, seq, x, y, confidence))
            elseif status == STATUS_GCS_DELIVERED then
                log_info(string.format("Jetson Bildirimi: %s (Seq: %d) - Bilgi amacli, aksiyon yok", status_name, seq))

            -- Aksiyon Tetikleyen Olaylar (3: GOTO_OBSERVATION, 4: ROUTE_READY)
            elseif status == STATUS_GOTO_OBSERVATION or status == STATUS_ROUTE_READY then
                latest_action_event = {
                    status     = status,
                    seq        = seq,
                    sess_id    = sess_id,
                    x          = x,
                    y          = y,
                    confidence = confidence
                }
                log_info(string.format("Aksiyon tetiklendi: %s (Seq: %d, Hedef: [%.1f, %.1f], Conf: %.2f)",
                    status_name, seq, x, y, confidence))
            end
        end

        ::continue_loop::
    end

    return latest_action_event
end

-- ============================================================================
-- MAVLINK BAŞLATMA (İLK YÜKLEME)
-- ============================================================================
if mavlink and mavlink.init then
    -- 20 mesajlık kuyruk derinliği, 1 adet kayıtlı msgid
    mavlink:init(20, 1)
    mavlink:register_rx_msgid(MAVLINK_MSG_ID_COMMAND_LONG)
    -- MAV_CMD_USER_1 komutunu bloke et: Otopilotun otomatik UNSUPPORTED ACK göndermesini engeller
    mavlink:block_command(MAV_CMD_USER_1)
    log_info("MAVLink arayuzu baslatildi (COMMAND_LONG dinleniyor, MAV_CMD_USER_1 bloke edildi).")
end

-- ============================================================================
-- ANA DÖNGÜ (UPDATE LOOP)
-- ============================================================================

local function update()
    local now_ms = get_time_ms()
    local current_mode = vehicle:get_mode()

    -- ------------------------------------------------------------------------
    -- GENEL KURAL 1: Pilot Müdahalesi Kontrolü
    -- BEKLEME, İNİŞ ve TAMAMLANDI dışındaki tüm uçuş durumlarında mod GUIDED
    -- olmak zorundadır. RC veya GCS'den mod değiştirilirse durum makinesi derhal durur.
    -- ------------------------------------------------------------------------
    if current_state ~= STATE_BEKLEME and 
       current_state ~= STATE_INIS and 
       current_state ~= STATE_TAMAMLANDI and 
       current_state ~= STATE_PILOT_MUDAHALESI then
        
        if current_mode ~= COPTER_MODE_GUIDED then
            log_warn(string.format("Pilot mudahalesi algilandi! Mod GUIDED disina alindi (Mod: %d). Gorev durduruldu.", current_mode))
            change_state(STATE_PILOT_MUDAHALESI)
            return update, UPDATE_RATE_MS
        end
    end

    -- ========================================================================
    -- 1. DURUM: BEKLEME
    -- Tetikleme (RC switch veya GUIDED moda geçiş) gelene kadar bekle.
    -- ========================================================================
    if current_state == STATE_BEKLEME then
        -- Yerde DISARMED iken MAVLink kuyrugunu tuket (SESSION_START el sikismasi ve canlilik)
        process_mavlink_queue(now_ms)

        local rc_triggered = false
        if rc and rc.get_aux_cached then
            -- Scripting 1 aux fonksiyonu (300) kontrol edilir
            local sw_pos = rc:get_aux_cached(300)
            if sw_pos and sw_pos == 2 then
                rc_triggered = true
            end
        end

        local guided_triggered = (current_mode == COPTER_MODE_GUIDED)

        if rc_triggered or guided_triggered then
            log_info(string.format("Gorev tetiklendi (Tetikleyici: %s). HAZIRLIK durumuna geciliyor.",
                guided_triggered and "GUIDED Mod" or "RC Anahtari"))
            hazirlik_samples = {}
            arm_requested = false
            change_state(STATE_HAZIRLIK)
        end

    -- ========================================================================
    -- 2. DURUM: HAZIRLIK
    -- - ahrs:get_relative_position_NED_origin() ile 5 örnek alıp ortalamasını kaydet.
    -- - Drone yerdeyken mevcut yaw açısını bir kez oku ve ILERI_YAW olarak sakla.
    -- - WP_SPD=1.0 ayarla ve doğrula (±0.05). Başarısızsa BEKLEME'ye dön.
    -- - GUIDED moda geç, arm et, KALKIŞ'a geç.
    -- ========================================================================
    elseif current_state == STATE_HAZIRLIK then
        -- Ornekleme sirasinda canlilik mesajlarini tuket (kuyruk tasmasini engelle)
        process_mavlink_queue(now_ms)

        -- A. Konum Örneklemesi
        local cur_pos = ahrs:get_relative_position_NED_origin()
        if not cur_pos then
            if (now_ms - state_entry_time_ms) > 15000 then
                log_error("EKF konumu alinamadi (15s zaman asimi)! BEKLEME durumuna donuluyor.")
                change_state(STATE_BEKLEME)
            end
            return update, UPDATE_RATE_MS
        end

        -- Sadece henüz 5 örneğe ulaşılmamışsa ve BASLANGIC_KONUMU hesaplanmamışsa örnek ekle
        if not BASLANGIC_KONUMU and #hazirlik_samples < 5 then
            table.insert(hazirlik_samples, { x = cur_pos:x(), y = cur_pos:y(), z = cur_pos:z() })
            log_info(string.format("HAZIRLIK: Konum ornegi alindi [%d/5]: Kuzey=%.2fm, Dogu=%.2fm, Z=%.2fm",
                #hazirlik_samples, cur_pos:x(), cur_pos:y(), cur_pos:z()))
            if #hazirlik_samples < 5 then
                -- 5 örnek toplanana kadar devam et
                return update, UPDATE_RATE_MS
            end
        end

        -- 5 örneğin ortalamasını hesapla (ASLA (0,0) varsayma)
        if not BASLANGIC_KONUMU then
            local sum_x, sum_y, sum_z = 0.0, 0.0, 0.0
            for _, s in ipairs(hazirlik_samples) do
                sum_x = sum_x + s.x
                sum_y = sum_y + s.y
                sum_z = sum_z + s.z
            end
            BASLANGIC_KONUMU = {
                kuzey = sum_x / 5.0,
                dogu  = sum_y / 5.0,
                z     = sum_z / 5.0
            }
            log_info(string.format("BASLANGIC_KONUMU kilitlendi: Kuzey=%.2fm, Dogu=%.2fm, Z=%.2fm (hazirlik_samples boyutu: %d, ornekleme durduruldu)",
                BASLANGIC_KONUMU.kuzey, BASLANGIC_KONUMU.dogu, BASLANGIC_KONUMU.z, #hazirlik_samples))
        end

        -- B. İleri Yaw Açısının Bir Kez Alınması (Fiziksel Kurulum Notu)
        if not ILERI_YAW then
            local yaw_rad = (ahrs.get_yaw_rad and ahrs:get_yaw_rad()) or ahrs:get_yaw()
            ILERI_YAW = yaw_rad
            log_info(string.format("ILERI_YAW kilitlendi: %.3f rad (%.1f deg) - Ilk yer acisi donduruldu.",
                ILERI_YAW, math.deg(ILERI_YAW)))
        end

        -- C. WP_SPD Parametre Ayarı ve Doğrulaması
        param:set("WP_SPD", 1.0)
        local read_spd = param:get("WP_SPD")
        if not read_spd or math.abs(read_spd - 1.0) > 0.05 then
            log_error(string.format("WP_SPD parametre dogrulamasi basarisiz! (Okunan: %s, Beklenen: 1.0 ±0.05) - Gorev iptal!",
                tostring(read_spd)))
            change_state(STATE_BEKLEME)
            return update, UPDATE_RATE_MS
        end

        -- D. GUIDED Mod ve Arm Kontrolü
        if current_mode ~= COPTER_MODE_GUIDED then
            if not vehicle:set_mode(COPTER_MODE_GUIDED) then
                log_error("GUIDED moda gecilemedi! Gorev iptal ediliyor.")
                change_state(STATE_BEKLEME)
                return update, UPDATE_RATE_MS
            end
        end

        if not arming:is_armed() then
            if not arm_requested then
                log_info("Drone arm ediliyor...")
                if not arming:arm() then
                    log_error("Arm komutu basarisiz! Pre-arm guvenlik kontrollerini kontrol edin.")
                    change_state(STATE_BEKLEME)
                    return update, UPDATE_RATE_MS
                end
                arm_requested = true
                arm_request_time_ms = now_ms
            end

            -- Arm teyidi için 5 saniye bekle
            if (now_ms - arm_request_time_ms) > 5000 then
                log_error("Arm teyidi zaman asimina ugradi (5s)! Gorev iptal ediliyor.")
                change_state(STATE_BEKLEME)
                return update, UPDATE_RATE_MS
            end
            return update, UPDATE_RATE_MS
        end

        -- Başarılı: Kalkış durumuna geç
        log_info("Hazirlik tamamlandi: WP_SPD=1.0m/s dogrulandi, GUIDED mod devrede, drone arm oldu.")
        kalkis_started = false
        kalkis_3d_steered = false
        change_state(STATE_KALKIS)

    -- ========================================================================
    -- 3. DURUM: KALKIŞ (Birleşik: 40m ileri + 33m irtifa, TEK hareket)
    -- - Hedef nokta:
    --   hedef_kuzey = BASLANGIC_KONUMU.kuzey + 40 * cos(ILERI_YAW)
    --   hedef_dogu  = BASLANGIC_KONUMU.dogu  + 40 * sin(ILERI_YAW)
    --   hedef_irtifa = 33.0m (Down: BASLANGIC_KONUMU.z - 33.0)
    -- - Kabul aralığı: yatay < 2m VE |irtifa - 33m| < 0.5m (31-35m bandı).
    -- - 2 saniye kararlılık sonrası HEDEF_BEKLE'ye geçilir.
    -- ========================================================================
    elseif current_state == STATE_KALKIS then
        -- Tirmanis boyunca gelen canlilik mesajlarini tuket (20 mesajlik kuyruk tasmasin)
        process_mavlink_queue(now_ms)

        kalkis_hedef_kuzey = BASLANGIC_KONUMU.kuzey + KALKIS_ILERELEME_M * math.cos(ILERI_YAW)
        kalkis_hedef_dogu  = BASLANGIC_KONUMU.dogu  + KALKIS_ILERELEME_M * math.sin(ILERI_YAW)
        kalkis_hedef_z     = BASLANGIC_KONUMU.z - HEDEF_IRTIFA_M

        -- Kalkış tırmanış motor profilini başlat
        if not kalkis_started then
            log_info(string.format("Birlesik 3D Kalkis baslatiliyor: 40m ileri (Pusula: %.1f deg), 33m irtifa. Hedef NED: [%.1f, %.1f, %.1f]",
                math.deg(ILERI_YAW), kalkis_hedef_kuzey, kalkis_hedef_dogu, kalkis_hedef_z))
            local takeoff_ok = vehicle:start_takeoff(HEDEF_IRTIFA_M)
            log_info(string.format("start_takeoff(%.1fm) cagirildi, donus degeri: %s", HEDEF_IRTIFA_M, tostring(takeoff_ok)))
            if not takeoff_ok then
                log_error("start_takeoff komutu reddedildi! Failsafe DONUS durumuna geciliyor.")
                change_state(STATE_DONUS)
                return update, UPDATE_RATE_MS
            end
            kalkis_started = true
        end

        -- Mevcut konumu, irtifayı ve tırmanış hızını kontrol et (eksik hız/konum asla 0.0 sayılmaz)
        local cur_pos = ahrs:get_relative_position_NED_origin()
        local cur_vel = ahrs:get_velocity_NED()
        if cur_pos and cur_vel then
            local rel_alt = BASLANGIC_KONUMU.z - cur_pos:z()
            local vz_up   = -cur_vel:z()
            local target_vec = make_vector3f(kalkis_hedef_kuzey, kalkis_hedef_dogu, kalkis_hedef_z)

            if not kalkis_3d_steered then
                -- İlk yönlendirme: Yerden ayrılma (irtifa >= 0.5m ve vz > 0.1m/s) teyit edilmelidir
                local is_airborne = (rel_alt >= 0.5 and vz_up > 0.1)
                if vehicle.get_likely_flying and not vehicle:get_likely_flying() then
                    is_airborne = false
                end

                if is_airborne then
                    local set_pos_ok = vehicle:set_target_pos_NED(target_vec, true, math.deg(ILERI_YAW), false, 0.0, false, false)
                    if set_pos_ok then
                        kalkis_3d_steered = true
                        log_info(string.format("Yerden ayrilma teyit edildi (Irtifa: %.2fm, Vz: %.2fm/s). Birlesik 3D rotaya gecildi (set_target_pos_NED kabul: true).",
                            rel_alt, vz_up))
                    else
                        log_warn(string.format("Yerden ayrilma teyit edildi (Irtifa: %.2fm, Vz: %.2fm/s), ancak set_target_pos_NED REDDEDILDI (donus: false). Tekrar denenecek.",
                            rel_alt, vz_up))
                    end
                end
            else
                -- Başarılı ilk yönlendirmeden sonra hedef komutu kalkis_3d_steered bayrağı ile kesintisiz sürdürülür
                vehicle:set_target_pos_NED(target_vec, true, math.deg(ILERI_YAW), false, 0.0, false, false)
            end

            local dx = cur_pos:x() - kalkis_hedef_kuzey
            local dy = cur_pos:y() - kalkis_hedef_dogu
            local horiz_dist = math.sqrt(dx*dx + dy*dy)
            local alt_err = math.abs(rel_alt - HEDEF_IRTIFA_M)

            -- Kabul kriterleri: yatay < 2m VE |irtifa - 33.0| < 0.5m (31-35m bandı içinde)
            if horiz_dist < KALKIS_YATAY_TOLERANS_M and 
               alt_err < IRTIFA_HATA_TOLERANS_M and 
               rel_alt >= IRTIFA_ALT_SINIR_M and 
               rel_alt <= IRTIFA_UST_SINIR_M then
                
                if not stability_start_ms then
                    stability_start_ms = now_ms
                    log_info(string.format("Kalkis hedef bandina girildi (Mesafe: %.2fm, Irtifa: %.2fm). 2s kararlilik bekleniyor...",
                        horiz_dist, rel_alt))
                elseif (now_ms - stability_start_ms) >= KARARLILIK_SURESI_MS then
                    log_info(string.format("Kalkis tamamlandi ve kararlilik saglandi (Mesafe: %.2fm, Irtifa: %.2fm). HEDEF_BEKLE durumuna geciliyor.",
                        horiz_dist, rel_alt))
                    change_state(STATE_HEDEF_BEKLE)
                    return update, UPDATE_RATE_MS
                end
            else
                stability_start_ms = nil
            end
        end

        -- Failsafe Zaman Aşımı (60 saniye)
        if (now_ms - state_entry_time_ms) > 60000 then
            log_warn("Kalkis 60s zaman asimi! Hedefe ulasilamadi, failsafe DONUS durumuna geciliyor.")
            change_state(STATE_DONUS)
            return update, UPDATE_RATE_MS
        end

    -- ========================================================================
    -- 4. DURUM: HEDEF_BEKLE
    -- - Girişte SEQ_ESIGI güncellenir.
    -- - MAVLink COMMAND_LONG dinlenir.
    -- - status == 3 -> HEDEF_KONUM sakla, HEDEFE_GİT'e geç.
    -- - status == 4 -> Doğrudan DÖNÜŞ'e geç.
    -- - 5s mesaj yoksa "link koptu" uyarısı ver.
    -- - 90s içinde 3 veya 4 gelmezse failsafe DÖNÜŞ'e geç.
    -- ========================================================================
    elseif current_state == STATE_HEDEF_BEKLE then
        -- 33m irtifada konum koruma komutunu tazele
        local hold_vec = make_vector3f(kalkis_hedef_kuzey, kalkis_hedef_dogu, BASLANGIC_KONUMU.z - HEDEF_IRTIFA_M)
        vehicle:set_target_pos_NED(hold_vec, true, math.deg(ILERI_YAW), false, 0.0, false, false)

        -- MAVLink mesajlarını tüket ve eylem olayını al
        local event = process_mavlink_queue(now_ms)
        if event then
            if event.status == STATUS_GOTO_OBSERVATION then
                HEDEF_KONUM = { x = event.x, y = event.y }
                log_info(string.format("GOTO_OBSERVATION alindi (Session ID: %d, Seq: %d, Hedef: [%.1f, %.1f], Conf: %.2f). HEDEFE_GIT durumuna geciliyor.",
                    event.sess_id or active_session_id or 0, event.seq, event.x, event.y, event.confidence))
                change_state(STATE_HEDEFE_GIT)
                return update, UPDATE_RATE_MS
            elseif event.status == STATUS_ROUTE_READY then
                log_info(string.format("ROUTE_READY alindi (Session ID: %d, Seq: %d). ROUTE_EXECUTE durumuna geciliyor.",
                    event.sess_id or active_session_id or 0, event.seq))
                mission_index = 1
                change_state(STATE_ROUTE_EXECUTE)
                return update, UPDATE_RATE_MS
            end
        end

        -- Bağlantı Canlılığı (Link Watchdog - 5 saniye)
        if last_msg_time_ms and (now_ms - last_msg_time_ms) > LINK_KOPMA_TIMEOUT_MS then
            if not link_lost_logged then
                log_warn(string.format("Jetson ile link koptu! (%.1fs suredir veri alinamadi, bekleniyor...)",
                    (now_ms - last_msg_time_ms) / 1000.0))
                link_lost_logged = true
            end
        end

        -- Failsafe Zaman Aşımı (90 saniye)
        if (now_ms - state_entry_time_ms) > TIMEOUT_HEDEF_BEKLE_MS then
            log_warn("HEDEF_BEKLE 90s zaman asimi! Ne status=3 ne status=4 alindi. Failsafe DONUS durumuna geciliyor.")
            change_state(STATE_DONUS)
            return update, UPDATE_RATE_MS
        end

    -- ========================================================================
    -- 5. DURUM: HEDEFE_GİT
    -- - HEDEF_KONUM'a (x, y) 33m sabit irtifada uç.
    -- - Yatay mesafe < 1m olunca 2 saniye kararlılık bekle, HEDEF_KONUMUNDA_BEKLE'ye geç.
    -- ========================================================================
    elseif current_state == STATE_HEDEFE_GIT then
        local target_vec = make_vector3f(HEDEF_KONUM.x, HEDEF_KONUM.y, BASLANGIC_KONUMU.z - HEDEF_IRTIFA_M)
        vehicle:set_target_pos_NED(target_vec, false, 0.0, false, 0.0, false, false)

        -- Uçuş sırasında gelen mesajları arka planda tüket (kuyruk şişmesin)
        process_mavlink_queue(now_ms)

        local cur_pos = ahrs:get_relative_position_NED_origin()
        if cur_pos then
            local dx = cur_pos:x() - HEDEF_KONUM.x
            local dy = cur_pos:y() - HEDEF_KONUM.y
            local horiz_dist = math.sqrt(dx*dx + dy*dy)

            if horiz_dist < GOZLEM_YATAY_TOLERANS_M then
                if not stability_start_ms then
                    stability_start_ms = now_ms
                    log_info(string.format("Gozlem noktasina varildi (Mesafe: %.2fm). 2s kararlilik bekleniyor...", horiz_dist))
                elseif (now_ms - stability_start_ms) >= KARARLILIK_SURESI_MS then
                    log_info(string.format("Gozlem noktasinda kararlilik saglandi (Mesafe: %.2fm). HEDEF_KONUMUNDA_BEKLE durumuna geciliyor.", horiz_dist))
                    change_state(STATE_HEDEF_KONUMUNDA_BEKLE)
                    return update, UPDATE_RATE_MS
                end
            else
                stability_start_ms = nil
            end
        end

        -- Failsafe Zaman Aşımı (120 saniye)
        if (now_ms - state_entry_time_ms) > TIMEOUT_GENEL_HAREKET_MS then
            log_warn("HEDEFE_GIT 120s zaman asimi! Hedefe varilamadi, failsafe DONUS durumuna geciliyor.")
            change_state(STATE_DONUS)
            return update, UPDATE_RATE_MS
        end

    -- ========================================================================
    -- 6. DURUM: HEDEF_KONUMUNDA_BEKLE
    -- - Girişte SEQ_ESIGI güncellenir.
    -- - status == 4 -> DÖNÜŞ'e geç.
    -- - status == 3 tekrar gelirse -> HEDEF_KONUM'u güncelle, HEDEFE_GİT'e geri dön.
    -- - 60s içinde status == 4 gelmezse failsafe DÖNÜŞ'e geç.
    -- ========================================================================
    elseif current_state == STATE_HEDEF_KONUMUNDA_BEKLE then
        -- Gözlem noktasında 33m irtifada sabit bekle
        local hold_vec = make_vector3f(HEDEF_KONUM.x, HEDEF_KONUM.y, BASLANGIC_KONUMU.z - HEDEF_IRTIFA_M)
        vehicle:set_target_pos_NED(hold_vec, false, 0.0, false, 0.0, false, false)

        local event = process_mavlink_queue(now_ms)
        if event then
            if event.status == STATUS_ROUTE_READY then
                log_info(string.format("ROUTE_READY alindi (Session ID: %d, Seq: %d). ROUTE_EXECUTE durumuna geciliyor.",
                    event.sess_id or active_session_id or 0, event.seq))
                mission_index = 1
                change_state(STATE_ROUTE_EXECUTE)
                return update, UPDATE_RATE_MS
            elseif event.status == STATUS_GOTO_OBSERVATION then
                HEDEF_KONUM = { x = event.x, y = event.y }
                log_info(string.format("Yeni GOTO_OBSERVATION alindi (Session ID: %d, Seq: %d, Hedef: [%.1f, %.1f], Conf: %.2f). HEDEFE_GIT durumuna geciliyor.",
                    event.sess_id or active_session_id or 0, event.seq, event.x, event.y, event.confidence))
                change_state(STATE_HEDEFE_GIT)
                return update, UPDATE_RATE_MS
            end
        end

        -- Bağlantı Canlılığı (Link Watchdog - 5 saniye)
        if last_msg_time_ms and (now_ms - last_msg_time_ms) > LINK_KOPMA_TIMEOUT_MS then
            if not link_lost_logged then
                log_warn(string.format("Jetson ile link koptu! (%.1fs suredir veri alinamadi, bekleniyor...)",
                    (now_ms - last_msg_time_ms) / 1000.0))
                link_lost_logged = true
            end
        end

        -- Failsafe Zaman Aşımı (60 saniye)
        if (now_ms - state_entry_time_ms) > TIMEOUT_KONUM_BEKLE_MS then
            log_warn("HEDEF_KONUMUNDA_BEKLE 60s zaman asimi! ROUTE_READY alinamadi, failsafe DONUS durumuna geciliyor.")
            change_state(STATE_DONUS)
            return update, UPDATE_RATE_MS
        end

    -- ========================================================================
    -- YENI DURUM: ROUTE_EXECUTE (SADECE DISARMED TEST)
    -- - mission:get_item() ile hedefleri cekip ekrana basar.
    -- - set_target_pos_NED YAPMAZ!
    -- ========================================================================
    elseif current_state == STATE_ROUTE_EXECUTE then
        local num_wp = mission:num_commands()
        if num_wp <= 0 then
            log_warn("HATA: Ardupilot hafizasinda mission kaydi yok. DONUS'e geciliyor.")
            change_state(STATE_DONUS)
            return update, UPDATE_RATE_MS
        end

        local wp_item = mission:get_item(mission_index)
        local frame = wp_item:frame() -- Orijinal frame'i okur
        local wp_x = wp_item:x() -- scaled lat if global frame
        local wp_y = wp_item:y() -- scaled lon
        local wp_z = wp_item:z() -- alt

        log_info(string.format("[TEST PING] WP[%d/%d] Geri Okundu: X/Lat: %d, Y/Lon: %d, Z/Alt: %.2f, Frame: %d", 
                  mission_index, num_wp-1, wp_x, wp_y, wp_z, frame))

        mission_index = mission_index + 1
        
        if mission_index >= num_wp then
            log_info(string.format("TEST PING TAMAMLANDI. (Toplam %d WP basariyla kalibre edildi) DONUS'e geciliyor.", num_wp))
            change_state(STATE_DONUS)
        end
        return update, 1000 -- Daha yavas dongu ile testi kalabalik yapmadan bitir

    -- ========================================================================
    -- 7. DURUM: DÖNÜŞ
    -- - BASLANGIC_KONUMU'na (HAZIRLIK'ta kaydedilen gerçek konum, asla (0,0) değil) uç.
    -- - Yatay mesafe < 1m olunca 2 saniye kararlılık bekle, İNİŞ'e geç.
    -- ========================================================================
    elseif current_state == STATE_DONUS then
        local return_vec = make_vector3f(BASLANGIC_KONUMU.kuzey, BASLANGIC_KONUMU.dogu, BASLANGIC_KONUMU.z - HEDEF_IRTIFA_M)
        vehicle:set_target_pos_NED(return_vec, false, 0.0, false, 0.0, false, false)

        -- Kuyruktaki mesajları tüket
        process_mavlink_queue(now_ms)

        local cur_pos = ahrs:get_relative_position_NED_origin()
        if cur_pos then
            local dx = cur_pos:x() - BASLANGIC_KONUMU.kuzey
            local dy = cur_pos:y() - BASLANGIC_KONUMU.dogu
            local horiz_dist = math.sqrt(dx*dx + dy*dy)

            if horiz_dist < DONUS_YATAY_TOLERANS_M then
                if not stability_start_ms then
                    stability_start_ms = now_ms
                    log_info(string.format("Kalkis noktasina yaklasildi (Mesafe: %.2fm). 2s kararlilik bekleniyor...", horiz_dist))
                elseif (now_ms - stability_start_ms) >= KARARLILIK_SURESI_MS then
                    log_info(string.format("Kalkis noktasinda kararlilik saglandi (Mesafe: %.2fm). INIS durumuna geciliyor.", horiz_dist))
                    change_state(STATE_INIS)
                    return update, UPDATE_RATE_MS
                end
            else
                stability_start_ms = nil
            end
        end

        -- Failsafe Zaman Aşımı (120 saniye)
        if (now_ms - state_entry_time_ms) > TIMEOUT_GENEL_HAREKET_MS then
            log_warn("DONUS 120s zaman asimi! INIS durumuna zorunlu geciliyor.")
            change_state(STATE_INIS)
            return update, UPDATE_RATE_MS
        end

    -- ========================================================================
    -- 8. DURUM: İNİŞ
    -- - LAND moduna geç (Mode 9).
    -- - Disarm teyidi al, "GÖREV TAMAMLANDI" logla.
    -- ========================================================================
    elseif current_state == STATE_INIS then
        if current_mode ~= COPTER_MODE_LAND then
            log_info("LAND moduna geciliyor...")
            vehicle:set_mode(COPTER_MODE_LAND)
        end

        -- Kuyruktaki mesajları tüket
        process_mavlink_queue(now_ms)

        -- Disarm kontrolü
        if not arming:is_armed() then
            log_info("GÖREV TAMAMLANDI - Drone basariyla indi ve disarm oldu.")
            change_state(STATE_TAMAMLANDI)
            return update, 1000
        end

    -- ========================================================================
    -- TAMAMLANDI VEYA PİLOT MÜDAHALESİ DURUMLARI
    -- ========================================================================
    elseif current_state == STATE_TAMAMLANDI or current_state == STATE_PILOT_MUDAHALESI then
        -- Script görevi bitirmiştir, sistemi yormamak için saniyede bir kez döner
        return update, 1000
    end

    return update, UPDATE_RATE_MS
end

-- ============================================================================
-- SCRIPT GİRİŞ NOKTASI
-- ============================================================================
log_info("S500 Otonom Gorev Scripti yuklendi. BEKLEME durumunda tetikleme bekleniyor...")

if _TEST_ENV then
    return {
        update = update,
        change_state = change_state,
        process_mavlink_queue = process_mavlink_queue,
        parse_command_long = parse_command_long,
        send_app_ack = send_app_ack,
        is_valid_confidence = is_valid_confidence,
        is_valid_coordinates = is_valid_coordinates,
        is_valid_raw_int = is_valid_raw_int,
        is_valid_raw_number = is_valid_raw_number,
        get_current_state = function() return current_state end,
        get_active_session_id = function() return active_session_id end,
        get_last_msg_time_ms = function() return last_msg_time_ms end,
        get_last_processed_seq = function() return last_processed_seq end,
        get_SEQ_ESIGI = function() return SEQ_ESIGI end,
        set_state = function(s) current_state = s; state_entry_time_ms = get_time_ms() end,
        set_state_entry_time_ms = function(t) state_entry_time_ms = t end,
        set_last_msg_time_ms = function(t) last_msg_time_ms = t end,
        set_last_processed_seq = function(s) last_processed_seq = s end,
        set_SEQ_ESIGI = function(s) SEQ_ESIGI = s end,
        get_start_record = function() return start_record end,
        get_goal_record = function() return goal_record end,
        get_accepted_messages = function() return accepted_messages end,
        get_retired_sessions = function() return retired_sessions end,
    }
end

return update()
