class Zero_Button_Event:
    def __init__(self):
        self._subscribers = []
        
    def subscribe(self, fn):
        self._subscribers.append(fn)
        
    def notify(self, *args, **kwargs):
        for fn in self._subscribers:
            fn(*args, **kwargs)