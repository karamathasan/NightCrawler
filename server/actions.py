import asyncio
from abc import ABC

class DXLActionBase(ABC):
    pass

class DXLAction(DXLActionBase):
    def __init__(self):
        pass
        self.timeout = 30
    
    def isDone(self):
        pass

class DXLActionGroup(DXLActionBase):
    def __init__(self, *actions:DXLAction):
        self.actions = actions
        # self.timeout = 30

    def isDone(self):
        for action in self.actions:
            if not action.isDone():
                return False
        return True
    
class ActionQueue():
    def __init__(self):
        self.queue = []

    def push(self, action: DXLAction):
        assert isinstance(action, DXLAction)
        self.queue.append(action)
    
    def pop(self):
        return self.queue.pop(0)