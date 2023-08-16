from luma.core.render import canvas
from abc import ABC, abstractmethod

class GUIElement(ABC):
    _xy : tuple[int, int]
    _width : int
    _height : int
    
    def __init__(self,
                 xy: tuple[int, int],
                 width: int,
                 height: int):
        self._xy = xy
        self._width = width
        self._height = height
    
    @abstractmethod
    def render(self, draw: canvas):
        pass