-- ============================================================================
-- İki Fazlı Kalkış (DIKEY_TIRMANIS & ILERI_HAREKET) İzole Doğrulama Testi
-- ============================================================================

local test_name = "Two-Phase Takeoff (DIKEY_TIRMANIS & ILERI_HAREKET) Unit Tests"
print("============================================================================")
print("BAŞLATILIYOR: " .. test_name)
print("============================================================================")

package.path = "/workspace/libraries/AP_Scripting/modules/?.lua;/workspace/sitl_test_run/scripts/modules/?.lua;" .. package.path

local mock_time_ms = 1000
local mock_is_armed = true
local mock_vehicle_mode = 4 -- GUIDED
local mock_cur_pos_x = 0.0
local mock_cur_pos_y = 0.0
local mock_cur_pos_z = 0.0
local mock_takeoff_calls = 0
local mock_target_pos_calls = {}

_G._TEST_ENV = true
_G.millis = function() return mock_time_ms end

_G.MAV_SEVERITY = {
    INFO = 6,
    WARNING = 4,
    ERROR = 3
}

_G.gcs = {
    send_text = function(self, sev, txt)
        -- print("[GCS]", sev, txt)
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
    start_takeoff = function(self, alt)
        mock_takeoff_calls = mock_takeoff_calls + 1
        return true
    end,
    set_target_pos_NED = function(self, pos, pos_valid, yaw, yaw_valid, yaw_rate, yaw_rate_valid, reset)
        table.insert(mock_target_pos_calls, { x = pos:x(), y = pos:y(), z = pos:z(), yaw = yaw })
        return true
    end,
    get_likely_flying = function() return true end
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
        v:x(0.0)
        v:y(0.0)
        v:z(-1.0) -- 1.0 m/s tırmanış
        return v
    end,
    get_yaw = function() return 0.0 end,
    get_yaw_rad = function() return 0.0 end
}

_G.param = {
    get = function(self, name) return 1.0 end,
    set = function(self, name, val) return true end
}

_G.mavlink = {
    init = function(self, d, c) end,
    register_rx_msgid = function(self, id) end,
    block_command = function(self, cmd) end,
    receive_chan = function(self) return nil, 0 end,
    send_chan = function(self, c, id, p) return true end
}
_G.mission = {
    num_commands = function() return 0 end
}

local mission = dofile("/workspace/s500_lua_mission/s500_mission.lua")
assert(type(mission) == "table", "s500_mission.lua yüklenemedi!")

-- Mock upvalue'ları bağla
local queued_mavlink_event = nil
local state_transitions = {}

local function setup_mission_upvalues(ground_pos, forward_yaw)
    local upval_i = 1
    while true do
        local name, val = debug.getupvalue(mission.update, upval_i)
        if not name then break end
        if name == "BASLANGIC_KONUMU" then
            debug.setupvalue(mission.update, upval_i, ground_pos)
        elseif name == "ILERI_YAW" then
            debug.setupvalue(mission.update, upval_i, forward_yaw)
        elseif name == "process_mavlink_queue" then
            debug.setupvalue(mission.update, upval_i, function(now_ms)
                local ev = queued_mavlink_event
                queued_mavlink_event = nil
                return ev
            end)
        elseif name == "change_state" then
            local orig_change = val
            debug.setupvalue(mission.update, upval_i, function(new_s)
                table.insert(state_transitions, new_s)
                return orig_change(new_s)
            end)
        end
        upval_i = upval_i + 1
    end
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
-- TEST 1: DIKEY_TIRMANIS İrtifa Bekleme ve Erken Geçmeme Kontrolü
-- ============================================================================
print("\n--- GRUP 1: DIKEY_TIRMANIS (Faz 1) Mantık Doğrulaması ---")
setup_mission_upvalues({ kuzey = 10.0, dogu = 20.0, z = 0.0 }, 0.0)
mock_time_ms = 1000
mission.set_state(3) -- STATE_DIKEY_TIRMANIS
mission.set_state_entry_time_ms(1000)

-- Araç henüz 15m irtifada (rel_alt = 15m, hedef 33m)
mock_cur_pos_x = 10.0
mock_cur_pos_y = 20.0
mock_cur_pos_z = -15.0

mission.update()
check("DIKEY_TIRMANIS sirasinda irtifa 15m iken durum degismemeli (State=3 kalmali)", mission.get_current_state() == 3)

-- ============================================================================
-- TEST 2: DIKEY_TIRMANIS 33m Varış ve 2s Kararlılıkla ILERI_HAREKET'e Geçiş
-- ============================================================================
mock_cur_pos_z = -33.0 -- Hedef irtifaya ulaşıldı
mock_time_ms = 2000
mission.update()
check("33m'ye ulasildigi ilk anda (henuz 2s dolmadi) durum State=3 kalmali", mission.get_current_state() == 3)

mock_time_ms = 4100 -- 2.1 saniye sonra
mission.update()
check("33m'de 2s kesintisiz kararlilik saglaninca ILERI_HAREKET (State=11) durumuna gecilmeli", mission.get_current_state() == 11)

-- ============================================================================
-- TEST 3: DIKEY_TIRMANIS 45s Zaman Aşımı Failsafe (Doğrudan INIS - State=8)
-- ============================================================================
mission.set_state(3)
mission.set_state_entry_time_ms(1000)
mock_cur_pos_z = -10.0 -- 10m'de takıldı
mock_time_ms = 46500 -- 45.5 saniye geçti (> 45s)
mission.update()
check("DIKEY_TIRMANIS 45s zaman asiminda guvenli sekilde INIS (State=8) durumuna gecilmeli", mission.get_current_state() == 8)

-- ============================================================================
-- TEST 4: ILERI_HAREKET (Faz 2) İleri Hedef Doğrulaması ve İlerleme
-- ============================================================================
print("\n--- GRUP 2: ILERI_HAREKET (Faz 2) Mantık Doğrulaması ---")
setup_mission_upvalues({ kuzey = 10.0, dogu = 20.0, z = 0.0 }, 0.0) -- Yaw=0 (tam Kuzey yönü)
mission.set_state(11) -- STATE_ILERI_HAREKET
mission.set_state_entry_time_ms(50000)
mock_time_ms = 50000
mock_cur_pos_x = 10.0 -- Henüz yerinde
mock_cur_pos_y = 20.0
mock_cur_pos_z = -33.0
mission.update()
check("ILERI_HAREKET durumuna gecildiginde hedef henuz uzakken State=11 kalmali", mission.get_current_state() == 11)

-- ============================================================================
-- TEST 5: ILERI_HAREKET İrtifa Güvenlik Bandı İhlali (< 31m) -> Doğrudan INIS (State=8)
-- ============================================================================
mock_cur_pos_z = -29.5 -- İrtifa 29.5m'ye düştü (< 31.0m ihlali)
mock_time_ms = 52000
mission.update()
check("ILERI_HAREKET sirasinda irtifa bandi ihlalinde (<31m) guvenli sekilde INIS (State=8) tetiklenmeli", mission.get_current_state() == 8)

-- ============================================================================
-- TEST 6: ILERI_HAREKET 60s Zaman Aşımı Failsafe -> Doğrudan INIS (State=8)
-- ============================================================================
mission.set_state(11)
mission.set_state_entry_time_ms(60000)
mock_cur_pos_x = 20.0 -- Hedef 43m (10 + 33), henuz 20m'de
mock_cur_pos_y = 20.0
mock_cur_pos_z = -33.0
mock_time_ms = 120500 -- 60.5 saniye geçti (> 60s)
mission.update()
check("ILERI_HAREKET 60s zaman asiminda hedefe varilamamissa guvenli sekilde INIS (State=8) tetiklenmeli", mission.get_current_state() == 8)

-- ============================================================================
-- TEST 7: ILERI_HAREKET Başarıyla 33m Varış, takeoff_return_point_ned Korunması ve HEDEF_BEKLE
-- ============================================================================
mission.set_takeoff_return_point_ned({ x = 10.0, y = 20.0, z = 0.0 })
mission.set_state(11)
mission.set_state_entry_time_ms(130000)
mock_time_ms = 130000
-- Hedefe ulaşıldı: x = 10 + 33 = 43.0m, y = 20.0m, z = -33.0m
mock_cur_pos_x = 43.0
mock_cur_pos_y = 20.0
mock_cur_pos_z = -33.0

mission.update()
check("Hedefe varildigi ilk anda (henuz 2s dolmadi) State=11 kalmali", mission.get_current_state() == 11)

mock_time_ms = 132100 -- 2.1 saniye sonra
mission.update()
local state_now = mission.get_current_state()
local ref_pt = mission.get_takeoff_return_point_ned()
check("33m ileride 2s kararlilik saglaninca HEDEF_BEKLE (State=4) durumuna gecilmeli", state_now == 4)
check("takeoff_return_point_ned 33m ileri konumuyla degistirilmemeli, baslangictaki zemin noktasini korumali",
    ref_pt ~= nil and math.abs(ref_pt.x - 10.0) < 0.01 and math.abs(ref_pt.y - 20.0) < 0.01 and math.abs(ref_pt.z - 0.0) < 0.01)

-- ============================================================================
-- TEST 8: STATE_DONUS Savunmacı Fallback Kontrolü (takeoff_return_point_ned nil iken çökme olmamalı)
-- ============================================================================
print("\n--- GRUP 3: STATE_DONUS Savunmacı Fallback Doğrulaması ---")
mission.set_takeoff_return_point_ned(nil) -- nil yap
mission.set_state(7) -- STATE_DONUS
mission.set_state_entry_time_ms(140000)
mock_time_ms = 140000
local ok_run, err_run = pcall(function() mission.update() end)
check("takeoff_return_point_ned == nil iken STATE_DONUS BASLANGIC_KONUMU fallback ile cokmeden calismali", ok_run, err_run)

-- takeoff_return_point_ned varken de dönüş noktasını doğru kullanmalı (fiziksel zemin başlangıç konumu: 10.0, 20.0)
mission.set_takeoff_return_point_ned({ x = 10.0, y = 20.0, z = 0.0 })
mock_target_pos_calls = {}
mission.update()
local last_target = mock_target_pos_calls[#mock_target_pos_calls]
check("takeoff_return_point_ned mevcutken STATE_DONUS hedefi kayitli zemin baslangic noktasi olmali",
    last_target ~= nil and math.abs(last_target.x - 10.0) < 0.01 and math.abs(last_target.y - 20.0) < 0.01)

-- ============================================================================
-- GRUP 4: HEDEF_BEKLE'den Doğrudan DONUS'a Geçiş (Status 4 / Bypass Doğrulaması)
-- ============================================================================
print("\n--- GRUP 4: HEDEF_BEKLE'den Dogrudan DONUS'a Gecis (Status 4 Bypass) ---")
state_transitions = {}
setup_mission_upvalues({ kuzey = 10.0, dogu = 20.0, z = 0.0 }, 0.0)
mission.set_takeoff_return_point_ned({ x = 10.0, y = 20.0, z = 0.0 })
mission.set_state(4) -- STATE_HEDEF_BEKLE
mission.set_state_entry_time_ms(150000)
mock_time_ms = 150000

-- STATUS_ROUTE_READY (status = 4, seq = 2, sess_id = 1001) enjekte et
queued_mavlink_event = { status = 4, seq = 2, sess_id = 1001 }

mission.update()
local state_after_route_ready = mission.get_current_state()

-- Eğer FSM ROUTE_EXECUTE (State 12) üzerinden DONUS'a (State 7) gidiyorsa o adımı da işlet
if state_after_route_ready == 12 then
    mission.update()
end
local final_state = mission.get_current_state()

check("HEDEF_BEKLE (State=4) durumundayken STATUS_ROUTE_READY alindiginda FSM STATE_DONUS (State=7) durumuna ulasmali", final_state == 7)

local visited_5_or_6 = false
for _, s in ipairs(state_transitions) do
    if s == 5 or s == 6 then
        visited_5_or_6 = true
        break
    end
end
check("Dogrudan geciste HEDEFE_GIT (State=5) veya HEDEF_KONUMUNDA_BEKLE (State=6) ara durumlarina kesinlikle ugranmamali", not visited_5_or_6)

print("\n============================================================================")
print(string.format("TEST SONUÇLARI: %d / %d BAŞARILI | 0 BAŞARISIZ", passed, total))
print("============================================================================")
os.exit(0)
