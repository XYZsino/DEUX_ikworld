"""Async memory behavior tree: RUNNING while awaited, bounded retry on Failure.

Cancellation and unexpected exceptions propagate; retry only explicit Failure.
"""
from enum import Enum

class Status(Enum):
    IDLE = 'IDLE'
    RUNNING = 'RUNNING'
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'

class Failure(ValueError): pass

class Leaf:
    def __init__(self, name, operation):
        self.name, self.operation, self.status = name, operation, Status.IDLE

    async def run(self):
        self.status = Status.RUNNING
        try:
            await self.operation()
            self.status = Status.SUCCESS
        except BaseException:
            self.status = Status.FAILURE
            raise

class Sequence(Leaf):
    def __init__(self, name, children):
        self.children = children
        super().__init__(name, self.tick_children)

    async def tick_children(self):
        for child in self.children: await child.run()

class Retry(Leaf):
    def __init__(self, name, child, attempts):
        if attempts < 1: raise ValueError('positive retry count required')
        self.child, self.attempts = child, attempts
        super().__init__(name, self.tick_child)

    async def tick_child(self):
        for attempt in range(self.attempts):
            try:
                await self.child.run()
                return
            except Failure:
                if attempt+1 == self.attempts: raise
