from luma.core.render import canvas
from PIL import ImageFont
# from . import GUIElement

class GUIElement():
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

    def render(self, draw: canvas):
        pass

class Button(GUIElement):
    _text: str
    _font: ImageFont
    
    def __init__(self, 
                 xy: tuple[int, int], 
                 width: int,
                 height: int,
                 text: str,
                 font: ImageFont = None) -> None:
        super().__init__(xy, width, height)
        
        self._text = text
        self._font = font
        
    def render(self, draw: canvas):
        bounding_box = draw.textbbox((10,10), self._text, font=self._font, align="center")
        padded_box = [bounding_box[0]-3, bounding_box[1]-3, bounding_box[2]+3, bounding_box[3]+3]
        draw.rectangle(padded_box)
        draw.text(self._xy, self._text, font=self._font)