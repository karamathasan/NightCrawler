from abc import ABC
import time

class ActionBase(ABC):
    def init(self):
        pass

    def execute(self):
        pass

    def end(self):
        pass

    def isDone(self):
        pass

class ActionGroup(ActionBase):
    def __init__(self, *actions:ActionBase):
        self.actions = actions

    def init(self):
        for action in self.actions:
            action.init()

    def execute(self):
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
    
    def clear(self):
        self.queue = []
    
class Wait(ActionBase):
    def __init__(self, time):
        self.wait_time = time

    def init(self):
        self.start = time.time()

    def isDone(self):
        return time.time()-self.start >= self.wait_time
    
class SequentialGroup(ActionGroup):
    def __init__(self, *actions: ActionBase):
        self.actions = [*actions]
        self.active = self.actions.pop(0)

    def init(self):
        self.active.init()

    def execute(self):
        if not self.active.isDone():
            self.active.execute()
        elif self.active.isDone():
            if len(self.actions) > 0:
                self.active = self.actions.pop(0)
                self.active.init()
        
    def isDone(self):
        return len(self.actions) == 0 and self.active.isDone()
    
class RepeatingGroup(ActionGroup):
    def __init__(self, *actions: ActionBase):
        super().__init__()
        self.actions = [*actions]
        self.position = 0
        self.active = self.actions[0]

    def init(self):
        self.active.init()

    def execute(self):
        if not self.active.isDone():
            self.active.execute()
        else:
            self.position = (self.position + 1) % len(self.actions)
            self.active = self.actions[self.position]
            self.active.init()
    
    def isDone(self):
        return False
    