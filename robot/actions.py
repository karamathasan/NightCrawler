from abc import ABC

class ActionBase(ABC):
    def init(self):
        pass

    def execute(self, dt):
        pass

    def end(self):
        pass

    def isDone(self):
        pass

class ActionGroup(ActionBase):
    def __init__(self, *actions:ActionBase):
        self.actions = actions
        # self.timeout = 30

    def init(self):
        for action in self.actions:
            action.init()

    def execute(self, dt):
        for action in self.actions:
            action.execute()

    def end(self):
        pass

    def isDone(self):
        for action in self.actions:
            if not action.isDone():
                return False
        return True
    
class ActionQueue():
    def __init__(self):
        self.queue = []
        self.active:ActionBase = None 

    def push(self, action: ActionBase):
        assert isinstance(action, ActionBase)
        self.queue.append(action)
    
    def pop(self)->ActionBase: 
        out = self.queue.pop(0)
        self.active = out
        return out
    
    def isEmpty(self):
        return (len(self.queue) == 0)
    

