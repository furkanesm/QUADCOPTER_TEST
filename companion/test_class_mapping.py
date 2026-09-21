import pytest
import sys
sys.path.append('c:/Users/Murat Ege/OneDrive/Masaüstü/DroneTest/QUADCOPTER_TEST/companion')
from class_mapping import ClassMapper

def test_class_mapper():
    starts = ['start', 's_class', 'kb_b']
    goals = ['hedef', 'h_class']
    mapper = ClassMapper(starts, goals)
    
    warns = []
    def log_warn(msg):
        warns.append(msg)
        
    assert mapper.get_msg_type('KB_B', log_warn) == 1
    assert mapper.get_msg_type('kb_b', log_warn) == 1
    assert mapper.get_msg_type(' KB_B ', log_warn) == 1
    assert len(warns) == 0
    
    assert mapper.get_msg_type('Hedef', log_warn) == 2
    assert mapper.get_msg_type('HEDEF', log_warn) == 2
    assert len(warns) == 0
    
    assert mapper.get_msg_type('engel', log_warn) == 0
    assert len(warns) == 0
    
    assert mapper.get_msg_type('bilinmeyen_x', log_warn) == 0
    assert len(warns) == 1
    
    assert mapper.get_msg_type('bilinmeyen_x', log_warn) == 0
    assert len(warns) == 1 # Does not log again
