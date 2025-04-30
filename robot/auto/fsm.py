from auto.state import State
class FiniteStateMachine():
    def __init__(self, init):
        self.current: State = init

    def update(self):
        if self.current.isDone():
            self.current.end()
            self.next()
        else:
            self.current.execute()

    def next(self):
        self.current = self.current.next()
        self.current.init()

