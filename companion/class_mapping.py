class ClassMapper:
    def __init__(self, start_classes, goal_classes):
        self.start_classes = [s.strip().lower() for s in start_classes]
        self.goal_classes = [s.strip().lower() for s in goal_classes]
        self.unknown_classes_logged = set()

    def get_msg_type(self, cls_name, logger_warn_fn):
        c_name = cls_name.strip().lower()
        if c_name in self.start_classes:
            return 1
        if c_name in self.goal_classes:
            return 2
        
        if c_name != 'engel' and c_name not in self.unknown_classes_logged:
            logger_warn_fn(f"[ADAPTER] Bilinmeyen sinif eslemesi, olay uretilmedi: '{cls_name}'")
            self.unknown_classes_logged.add(c_name)
        
        return 0
