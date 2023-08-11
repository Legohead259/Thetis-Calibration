from enum import Enum
from luma.core.render import canvas
from math import floor

class BarStyle(Enum):
    SOLID             = 0
    DIAGONAL_FORWARD  = 1
    DIAGONAL_BACKWARD = 2
    ARROW_FORWARD     = 3
    ARROW_BACKWARD    = 4
    

class BarBase():
    _init : bool
    _x : int
    _y : int
    _width : int
    _height : int
    _canvas : canvas
    _style : BarStyle
    _band_width : int
    _percent : float
    
    def __init__(self, x: int, y: int, width: int, height: int, canvas: canvas, style: BarStyle=0, band_width: int=20, percent: float=0):
        self._init = False
        
        self._x = x
        self._y = y
        self._width = width
        self._height = height
        self._canvas = canvas
        self._style = style
        self._band_width = band_width
        self.percent = percent
        
        self.update()
        self._init = True
        
    def update(self):
        bar_width = floor(self._width * (self._percent / 100.0))
        self._canvas.rectangle((self._x, self._y, self._x+bar_width, self._y+self._height), 
                               outline="white", 
                               fill="white")        
        if not self._init:
            self._canvas.rectangle((self._x, self._y, self._x+self._width, self._y+self._height), 
                                   outline="white", 
                                   fill="black")

    @property
    def percent(self):
        return self._percent
    
    @percent.setter
    def percent(self, value: float):
        if value > 100:
            value = 100
        elif value < 0:
            value = 0
            
        self._percent = value
        self.update()