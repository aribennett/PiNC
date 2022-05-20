
class State(object):
    def __init__(self):
        self.event_map = {'abort': AbortState}

    def run(self):
        pass

    def on_event(self, event):
        if event in self.event_map:
            return self.event_map[event]()
        else:
            return self


class AbortState(State):
    def __init__(self):
        super().__init__()