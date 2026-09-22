#!/usr/bin/env python3
"""
Unit test suite for the hardened MavlinkAdapterNode and RobustStatustextAssembler.
Directly imports companion/mavlink_mission_commander.py.

Tests:
1. RobustStatustextAssembler missing middle chunk & completeness (0..final_seq)
2. STATUSTEXT source filtering (sysid/compid verification)
3. ACK -> Log order with successful_txs preserved in acked_commands
4. Log -> ACK order with early transition seen, retry_loop survival, and delayed ACK correlation
5. Status 1 and Status 2 ACK behavior (record/ACK only, no state transition tracking)
6. Early log with missing ACK timeout (finite wait, no infinite pending, no auto-resend)
7. ACKED_TRANSITION_PENDING timeout and late log rejection (no late confirmation)
8. Cross-session ambiguity rejection (session change / epoch > 1 prevents false correlation)
9. Failed transmission tracking (send_user_cmd failure does NOT enable log correlation)
"""

import os
import sys
import time
import math
from unittest.mock import MagicMock, patch

# Ensure companion dir is in path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mavlink_mission_commander import MavlinkAdapterNode, RobustStatustextAssembler


class MockMavMsg:
    def __init__(self, text, src_sys=1, src_comp=1, chunk_id=0, chunk_seq=0, msg_type="STATUSTEXT"):
        self._type = msg_type
        self.srcSystem = src_sys
        self.srcComponent = src_comp
        self.id = chunk_id
        self.chunk_seq = chunk_seq
        if isinstance(text, bytes):
            self._text_raw = text
            self.text = text
        else:
            self._text_raw = text.encode('utf-8')
            self.text = text

    def get_type(self):
        return self._type

    def get_srcSystem(self):
        return self.srcSystem

    def get_srcComponent(self):
        return self.srcComponent


class MockAckMsg:
    def __init__(self, status, result, seq, session_id, src_sys=1, src_comp=1):
        self._type = "COMMAND_LONG"
        self.srcSystem = src_sys
        self.srcComponent = src_comp
        self.param1 = float(status)
        self.param2 = float(result)
        self.param3 = 0.0
        self.param4 = float(seq)
        self.param5 = float(0xAB)
        self.param6 = float(session_id)
        self.param7 = 1.0

    def get_type(self):
        return self._type

    def get_srcSystem(self):
        return self.srcSystem

    def get_srcComponent(self):
        return self.srcComponent


def create_test_node():
    if not rclpy.ok():
        rclpy.init()
    with patch('pymavlink.mavutil.mavlink_connection') as mock_conn:
        mock_master = MagicMock()
        mock_conn.return_value = mock_master
        node = MavlinkAdapterNode()
        events = []
        node.pub_event_status.publish = lambda msg: events.append(msg.data if hasattr(msg, 'data') else str(msg))
        return node, events


def test_1_assembler_missing_middle_chunk():
    """1. Eksik orta parça ve 0..final_seq tamlık kontrolü."""
    asm = RobustStatustextAssembler(timeout_s=2.0)
    now = 100.0

    # 3 parçalı mesaj: chunk 0, chunk 1, chunk 2 (son parça, len < 50)
    chunk0_text = ("A" * 50).encode('utf-8')
    chunk1_text = ("B" * 50).encode('utf-8')
    chunk2_text = b"C" * 10  # is_last = True

    m0 = MockMavMsg(chunk0_text, chunk_id=1, chunk_seq=0)
    m1 = MockMavMsg(chunk1_text, chunk_id=1, chunk_seq=1)
    m2 = MockMavMsg(chunk2_text, chunk_id=1, chunk_seq=2)

    # Chunk 0 geldi -> Henüz son parça yok, None dönmeli
    res0 = asm.feed(m0, now)
    assert res0 is None, "Chunk 0 tek başına metin döndürmemelidir"

    # Chunk 2 (son parça) geldi, ama chunk 1 EKSİK!
    # final_seq=2 oldu fakat chunk 1 bulunmadığı için çıktı döndürmemeli!
    res2 = asm.feed(m2, now + 0.1)
    assert res2 is None, "Eksik parça (chunk 1) varken son parça gelse bile çıktı DÖNDÜRMEMELİDİR!"

    # Şimdi eksik olan Chunk 1 ulaştı: 0, 1, 2 tamamlandı!
    res1 = asm.feed(m1, now + 0.2)
    assert res1 is not None, "Tüm parçalar (0..final_seq) tamamlandığında çıktı dönmelidir"
    expected = ("A" * 50) + ("B" * 50) + ("C" * 10)
    assert res1 == expected, f"Reassembled text mismatch: got {res1}, expected {expected}"
    print("✓ Test 1: RobustStatustextAssembler eksik orta parça durumunda tamlık kontrolü doğrulandı.")


def test_2_statustext_source_filter():
    """2. STATUSTEXT kaynak filtresi (sysid/compid dışından gelenler elenmeli)."""
    node, events = create_test_node()
    node.session_state = node.ACTIVE
    node.session_epoch = 1
    node.lua_fsm_state = "BEKLEME"

    # Yabancı sistemden (sys_id=2) gelen FSM durum logu
    foreign_msg = MockMavMsg("[S500 LUA] Durum Gecisi: BEKLEME -> HEDEF_BEKLE", src_sys=2, src_comp=1)
    node._handle_statustext_msg(foreign_msg)
    assert node.lua_fsm_state == "BEKLEME", "Yabancı sysid mesajı FSM durumunu DEĞİŞTİRMEMELİDİR!"

    # Doğru otopilottan (sys_id=1, comp_id=1) gelen durum logu
    valid_msg = MockMavMsg("[S500 LUA] Durum Gecisi: BEKLEME -> HEDEF_BEKLE", src_sys=1, src_comp=1)
    node._handle_statustext_msg(valid_msg)
    assert node.lua_fsm_state == "HEDEF_BEKLE", "Geçerli otopilot mesajı FSM durumunu güncellemelidir!"
    assert any("OBSERVED_STATE:HEDEF_BEKLE" in e for e in events)
    print("✓ Test 2: STATUSTEXT kaynak filtrelemesi (sysid/compid) başarıyla doğrulandı.")


def test_3_ack_then_log_with_successful_txs():
    """3. Başarılı gönderim → ACK → Log sırası ve successful_txs aktarımı."""
    node, events = create_test_node()
    node.session_state = node.ACTIVE
    node.session_epoch = 1
    node.lua_fsm_state = "HEDEF_BEKLE"

    # Status 3 kuyruğa al
    now_ros = node.get_clock().now()
    seq = node.queue_command(3, 10.0, 5.0, 1.0, now_ros, allowed_states=node.ALLOWED_STATES_GOTO_OBS)
    assert seq == 1

    # Başarılı gönderim simülasyonu
    node.pending_commands[seq]['successful_txs'] = 1
    node.pending_commands[seq]['first_tx_mono'] = 100.0
    node.pending_commands[seq]['retries'] = 1

    # 1. ACK geldi
    ack_msg = MockAckMsg(status=3, result=0, seq=seq, session_id=node.session_id)
    node._handle_ack_msg(ack_msg)

    # successful_txs ACK tablosuna geçmiş olmalı!
    assert seq in node.acked_commands, "Komut acked_commands tablosuna taşınmalıdır"
    assert node.acked_commands[seq]['successful_txs'] == 1, "successful_txs ACK tablosuna AKTARILMALIDIR!"
    assert node.acked_commands[seq]['transition_state'] == 'ACKED_TRANSITION_PENDING'
    assert any(f"ACCEPTED:{seq}:3" in e for e in events)
    assert any(f"ACKED_TRANSITION_PENDING:{seq}:3" in e for e in events)

    # 2. Geçiş logu geldi
    log_msg = MockMavMsg(f"[S500 LUA] GOTO_OBSERVATION alindi (Seq: {seq}, Hedef: [10.0, 5.0], Conf: 1.00). HEDEFE_GIT durumuna geciliyor.")
    node._handle_statustext_msg(log_msg)

    assert node.acked_commands[seq]['transition_state'] == 'TRANSITION_CONFIRMED'
    assert any(f"CORRELATED_TRANSITION:HEDEFE_GIT:{seq}:3" in e for e in events)
    print("✓ Test 3: ACK -> Log sırası ve successful_txs aktarımı başarıyla doğrulandı.")


def test_4_log_then_ack_and_retry_loop_survival():
    """4. Başarılı gönderim → Erken Log & Durum Değişikliği → retry_loop → Gecikmiş ACK sırası."""
    node, events = create_test_node()
    node.session_state = node.ACTIVE
    node.session_epoch = 1
    node.lua_fsm_state = "HEDEF_BEKLE"

    seq = node.queue_command(3, 12.0, 8.0, 1.0, node.get_clock().now(), allowed_states=node.ALLOWED_STATES_GOTO_OBS)
    node.pending_commands[seq]['successful_txs'] = 1
    node.pending_commands[seq]['first_tx_mono'] = 100.0
    node.pending_commands[seq]['retries'] = 1

    # 1. Erken eylem logu ve FSM durum geçişi logu geldi (ACK'den önce)
    action_log = MockMavMsg(f"[S500 LUA] GOTO_OBSERVATION alindi (Seq: {seq}, Hedef: [12.0, 8.0], Conf: 1.00). HEDEFE_GIT durumuna geciliyor.")
    state_log = MockMavMsg("[S500 LUA] Durum Gecisi: HEDEF_BEKLE -> HEDEFE_GIT")
    node._handle_statustext_msg(action_log)
    node._handle_statustext_msg(state_log)

    assert node.pending_commands[seq]['early_transition_seen'] is True
    assert node.lua_fsm_state == "HEDEFE_GIT"
    assert any(f"EARLY_TRANSITION_OBSERVED:HEDEFE_GIT:{seq}:3" in e for e in events)

    # 2. retry_loop çalışıyor: FSM HEDEFE_GIT olduğu halde komut SİLİNMEMELİ ve TEKRAR GÖNDERİLMEMELİ!
    node.send_user_cmd = MagicMock(return_value=True)
    node.retry_loop()

    assert seq in node.pending_commands, "Erken logu görülmüş komut retry_loop tarafından silinmemelidir!"
    node.send_user_cmd.assert_not_called()  # Komut tekrar gönderilmemeli!

    # 3. Gecikmiş ACK ulaştı
    ack_msg = MockAckMsg(status=3, result=0, seq=seq, session_id=node.session_id)
    node._handle_ack_msg(ack_msg)

    assert seq in node.acked_commands
    assert node.acked_commands[seq]['transition_state'] == 'TRANSITION_CONFIRMED'
    assert any(f"CORRELATED_TRANSITION:HEDEFE_GIT:{seq}:3" in e for e in events)
    print("✓ Test 4: Log -> ACK sırası, retry_loop koruması ve gecikmiş ACK teyidi başarıyla doğrulandı.")


def test_5_status12_ack_behavior():
    """5. Status 1/2 ACK işlemleri: Yalnızca kayıt/ACK, geçiş tablosuna taşınmaz."""
    node, events = create_test_node()
    node.session_state = node.ACTIVE
    node.session_epoch = 1

    seq1 = node.queue_command(1, 5.0, 5.0, 0.9, node.get_clock().now())
    seq2 = node.queue_command(2, 15.0, 15.0, 0.95, node.get_clock().now())

    # Status 1 ACK
    ack1 = MockAckMsg(status=1, result=0, seq=seq1, session_id=node.session_id)
    node._handle_ack_msg(ack1)
    assert any(f"ACCEPTED:{seq1}:1" in e for e in events)
    assert seq1 not in node.pending_commands
    assert seq1 not in node.acked_commands, "Status 1 acked_commands içine EKLENMEMELİDİR!"
    assert not any(f"ACKED_TRANSITION_PENDING:{seq1}:1" in e for e in events)

    # Status 2 ACK
    ack2 = MockAckMsg(status=2, result=0, seq=seq2, session_id=node.session_id)
    node._handle_ack_msg(ack2)
    assert any(f"ACCEPTED:{seq2}:2" in e for e in events)
    assert seq2 not in node.pending_commands
    assert seq2 not in node.acked_commands, "Status 2 acked_commands içine EKLENMEMELİDİR!"
    assert not any(f"ACKED_TRANSITION_PENDING:{seq2}:2" in e for e in events)
    print("✓ Test 5: Status 1 ve 2 ACK davranışı (state geçişi beklenmeme) başarıyla doğrulandı.")


def test_6_early_log_missing_ack_timeout():
    """6. Erken log alınıp ACK hiç gelmediğinde sınırlı bekleme ve zaman aşımı."""
    node, events = create_test_node()
    node.session_state = node.ACTIVE
    node.session_epoch = 1
    node.transition_timeout = 2.0

    seq = node.queue_command(3, 10.0, 10.0, 1.0, node.get_clock().now())
    node.pending_commands[seq]['successful_txs'] = 1
    node.pending_commands[seq]['first_tx_mono'] = 100.0

    # Erken log geldi
    log_msg = MockMavMsg(f"[S500 LUA] GOTO_OBSERVATION alindi (Seq: {seq}, Hedef: [10.0, 10.0], Conf: 1.00). HEDEFE_GIT durumuna geciliyor.")
    node._handle_statustext_msg(log_msg)
    assert node.pending_commands[seq]['early_transition_seen'] is True

    # ACK gelmedi, aradan 3 saniye geçti (timeout=2.0)
    fake_now = time.monotonic() + 3.0
    with patch('time.monotonic', return_value=fake_now):
        node.retry_loop()

    assert seq not in node.pending_commands, "ACK zaman aşımına uğrayan kayıt pending kuyruğundan silinmelidir!"
    assert any(f"EARLY_TRANSITION_ACK_TIMEOUT_UNVERIFIED:3:seq_{seq}" in e for e in events)
    print("✓ Test 6: Erken log sonrası ACK gelmeme zaman aşımı başarıyla doğrulandı.")


def test_7_acked_transition_timeout_and_late_log():
    """7. ACKED_TRANSITION_PENDING zaman aşımı ve geç logun sonradan CONFIRMED yapmaması."""
    node, events = create_test_node()
    node.session_state = node.ACTIVE
    node.session_epoch = 1
    node.transition_timeout = 2.0

    seq = node.queue_command(3, 14.0, 7.0, 1.0, node.get_clock().now())
    node.pending_commands[seq]['successful_txs'] = 1
    node.pending_commands[seq]['first_tx_mono'] = 100.0

    # ACK geldi
    ack_msg = MockAckMsg(status=3, result=0, seq=seq, session_id=node.session_id)
    node._handle_ack_msg(ack_msg)
    assert node.acked_commands[seq]['transition_state'] == 'ACKED_TRANSITION_PENDING'

    # Geçiş logu gelmedi, zaman aşımı doldu
    fake_now = time.monotonic() + 3.0
    with patch('time.monotonic', return_value=fake_now):
        node.retry_loop()

    assert node.acked_commands[seq]['transition_state'] == 'TRANSITION_TIMEOUT_UNVERIFIED'
    assert any(f"TRANSITION_TIMEOUT_UNVERIFIED:3:seq_{seq}" in e for e in events)

    # Şimdi GEÇ bir log ulaştı: Kesin başarıya DÖNÜŞMEMELİ, geç gözlem olarak raporlanmalı!
    late_log = MockMavMsg(f"[S500 LUA] GOTO_OBSERVATION alindi (Seq: {seq}, Hedef: [14.0, 7.0], Conf: 1.00). HEDEFE_GIT durumuna geciliyor.")
    node._handle_statustext_msg(late_log)

    assert node.acked_commands[seq]['transition_state'] == 'TRANSITION_TIMEOUT_UNVERIFIED', "Zaman aşımına uğramış kayıt CONFIRMED yapılmamalıdır!"
    assert any(f"LATE_TRANSITION_LOG_AFTER_TIMEOUT:3:seq_{seq}" in e for e in events)
    assert not any(f"CORRELATED_TRANSITION:HEDEFE_GIT:{seq}:3" in e for e in events)
    print("✓ Test 7: ACK geçiş zaman aşımı ve geç log reddi başarıyla doğrulandı.")


def test_8_cross_session_ambiguity_rejection():
    """8. Oturum değişimi sonrası eski log belirsizliği (TRANSITION_CONFIRMED engellenmeli)."""
    node, events = create_test_node()
    node.session_state = node.ACTIVE
    node.session_epoch = 1

    # 1. Oturum durduruluyor
    node.local_stop("First session ended")

    # 2. Oturum başlatılıyor (epoch = 2)
    req = Trigger.Request()
    res = Trigger.Response()
    node.srv_start_cb(req, res)
    assert node.session_epoch == 2
    node.session_state = node.ACTIVE

    # 2. Oturumda henüz bir seq=1 komutu gönderildi diyelim
    seq = node.queue_command(3, 20.0, 20.0, 1.0, node.get_clock().now())
    node.pending_commands[seq]['successful_txs'] = 1
    node.pending_commands[seq]['first_tx_mono'] = 100.0

    # Ancak Lua logunda session_id bulunmadığı için epoch > 1 durumunda korelasyon belirsizdir!
    log_msg = MockMavMsg(f"[S500 LUA] GOTO_OBSERVATION alindi (Seq: {seq}, Hedef: [20.0, 20.0], Conf: 1.00). HEDEFE_GIT durumuna geciliyor.")
    node._handle_statustext_msg(log_msg)

    assert not any(f"CORRELATED_TRANSITION:HEDEFE_GIT:{seq}:3" in e for e in events)
    assert any(f"AMBIGUOUS_SESSION_LOG_IGNORED:3:seq_{seq}:session_epoch_2_lacks_session_in_log" in e for e in events)
    print("✓ Test 8: Oturum değişimi sonrası eski log belirsizliği (ambiguity rejection) başarıyla doğrulandı.")


def test_9_failed_tx_tracking():
    """9. Gönderim başarısız olduğunda log geçiş doğrulamasına dönüşmemelidir."""
    node, events = create_test_node()
    node.session_state = node.ACTIVE
    node.session_epoch = 1

    seq = node.queue_command(3, 22.0, 11.0, 1.0, node.get_clock().now())
    # Başarısız gönderim: successful_txs = 0, first_tx_mono = 0.0
    node.pending_commands[seq]['retries'] = 1
    node.pending_commands[seq]['successful_txs'] = 0
    node.pending_commands[seq]['first_tx_mono'] = 0.0

    # Bu komut için log geldiği iddia edilirse
    log_msg = MockMavMsg(f"[S500 LUA] GOTO_OBSERVATION alindi (Seq: {seq}, Hedef: [22.0, 11.0], Conf: 1.00). HEDEFE_GIT durumuna geciliyor.")
    node._handle_statustext_msg(log_msg)

    assert node.pending_commands[seq]['early_transition_seen'] is False
    assert any(f"AMBIGUOUS_SESSION_LOG_IGNORED:3:seq_{seq}:not_transmitted_yet" in e for e in events)
    print("✓ Test 9: Başarısız gönderimin log korelasyonuna dönüşmesi başarıyla engellendi.")


def test_10_session_id_log_verification_matrix():
    """10. Oturum doğrulaması matrisi: (a) ID mevcut ve eşleşiyor, (b) ID mevcut ama uyumsuz, (c) ID logda yok."""
    node, events = create_test_node()
    node.session_state = node.ACTIVE
    node.session_epoch = 2  # 2. oturum (yeni epoch)

    # -------------------------------------------------------------------------
    # (a) Status 3 ve 4: Log'da Session ID mevcut ve eşleşiyor -> CONFIRMED
    # -------------------------------------------------------------------------
    # Status 3 (GOTO_OBSERVATION)
    seq3 = node.queue_command(3, 25.0, 15.0, 1.0, node.get_clock().now())
    node.pending_commands[seq3]['successful_txs'] = 1
    node.pending_commands[seq3]['first_tx_mono'] = 100.0
    ack3 = MockAckMsg(status=3, result=0, seq=seq3, session_id=node.session_id)
    node._handle_ack_msg(ack3)

    log_match_s3 = MockMavMsg(f"[S500 LUA] GOTO_OBSERVATION alindi (Session ID: {node.session_id}, Seq: {seq3}, Hedef: [25.0, 15.0], Conf: 1.00). HEDEFE_GIT durumuna geciliyor.")
    node._handle_statustext_msg(log_match_s3)
    assert node.acked_commands[seq3]['transition_state'] == 'TRANSITION_CONFIRMED'
    assert any(f"CORRELATED_TRANSITION:HEDEFE_GIT:{seq3}:3" in e for e in events)

    # Status 4 (ROUTE_READY)
    node.lua_fsm_state = "HEDEF_BEKLE"
    seq4 = node.queue_command(4, 0.0, 0.0, 1.0, node.get_clock().now(), allowed_states=node.ALLOWED_STATES_ROUTE_READY)
    node.pending_commands[seq4]['successful_txs'] = 1
    node.pending_commands[seq4]['first_tx_mono'] = 105.0
    ack4 = MockAckMsg(status=4, result=0, seq=seq4, session_id=node.session_id)
    node._handle_ack_msg(ack4)

    log_match_s4 = MockMavMsg(f"[S500 LUA] ROUTE_READY alindi (Session ID: {node.session_id}, Seq: {seq4}). Dogrudan DONUS durumuna geciliyor.")
    node._handle_statustext_msg(log_match_s4)
    assert node.acked_commands[seq4]['transition_state'] == 'TRANSITION_CONFIRMED'
    assert any(f"CORRELATED_TRANSITION:DONUS:{seq4}:4" in e for e in events)

    # -------------------------------------------------------------------------
    # (b) Log'da Session ID mevcut ama eşleşmiyor (mismatch) -> REDDEDİLMELİ
    # -------------------------------------------------------------------------
    seq_mismatch = node.queue_command(3, 30.0, 20.0, 1.0, node.get_clock().now())
    node.pending_commands[seq_mismatch]['successful_txs'] = 1
    node.pending_commands[seq_mismatch]['first_tx_mono'] = 110.0
    ack_m = MockAckMsg(status=3, result=0, seq=seq_mismatch, session_id=node.session_id)
    node._handle_ack_msg(ack_m)

    log_mismatch = MockMavMsg(f"[S500 LUA] GOTO_OBSERVATION alindi (Session ID: 888888, Seq: {seq_mismatch}, Hedef: [30.0, 20.0], Conf: 1.00). HEDEFE_GIT durumuna geciliyor.")
    node._handle_statustext_msg(log_mismatch)
    assert node.acked_commands[seq_mismatch]['transition_state'] == 'ACKED_TRANSITION_PENDING'
    assert any(f"AMBIGUOUS_SESSION_LOG_IGNORED:3:seq_{seq_mismatch}:log_session_888888_mismatch_current_{node.session_id}" in e for e in events)

    # -------------------------------------------------------------------------
    # (c) Log'da Session ID hiç yok ve epoch > 1 -> REDDEDİLMELİ (lacks_session)
    # -------------------------------------------------------------------------
    seq_no_sess = node.queue_command(3, 35.0, 25.0, 1.0, node.get_clock().now())
    node.pending_commands[seq_no_sess]['successful_txs'] = 1
    node.pending_commands[seq_no_sess]['first_tx_mono'] = 115.0
    ack_no = MockAckMsg(status=3, result=0, seq=seq_no_sess, session_id=node.session_id)
    node._handle_ack_msg(ack_no)

    log_legacy = MockMavMsg(f"[S500 LUA] GOTO_OBSERVATION alindi (Seq: {seq_no_sess}, Hedef: [35.0, 25.0], Conf: 1.00). HEDEFE_GIT durumuna geciliyor.")
    node._handle_statustext_msg(log_legacy)
    assert node.acked_commands[seq_no_sess]['transition_state'] == 'ACKED_TRANSITION_PENDING'
    assert any(f"AMBIGUOUS_SESSION_LOG_IGNORED:3:seq_{seq_no_sess}:session_epoch_2_lacks_session_in_log" in e for e in events)
    print("✓ Test 10: Oturum doğrulama matrisi ((a) eşleşme onayı, (b) uyumsuzluk reddi, (c) eksik ID fallback) başarıyla doğrulandı.")


def run_tests():
    print("=== MavlinkAdapterNode & Statustext Hardened Unit Tests Başlatılıyor ===")
    test_1_assembler_missing_middle_chunk()
    test_2_statustext_source_filter()
    test_3_ack_then_log_with_successful_txs()
    test_4_log_then_ack_and_retry_loop_survival()
    test_5_status12_ack_behavior()
    test_6_early_log_missing_ack_timeout()
    test_7_acked_transition_timeout_and_late_log()
    test_8_cross_session_ambiguity_rejection()
    test_9_failed_tx_tracking()
    test_10_session_id_log_verification_matrix()
    print("\n>>> BÜTÜN BİRİM TESTLER EKSİKSİZ GEÇTİ (10/10) <<<")


if __name__ == '__main__':
    run_tests()
