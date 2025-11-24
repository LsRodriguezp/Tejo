## -------------------------------------------------------------------------
## @author Sofía
## Basado en BaseListener de Leonardo Florez-Valencia
## -------------------------------------------------------------------------

import Ogre
import Ogre.Bites as OgreBites
import math

class TejoListener(OgreBites.InputListener):

    def __init__(self, game_ref):
        super().__init__()
        self.game = game_ref
        self.shift_pressed = False
        self.mouse_down_time = None

    def keyPressed(self, evt):
        sym = evt.keysym.sym
        mod = evt.keysym.mod

        # Detectar SHIFT
        if sym == OgreBites.SDLK_LSHIFT or sym == OgreBites.SDLK_RSHIFT:
            self.shift_pressed = True

        # Escape: salir
        elif sym == OgreBites.SDLK_ESCAPE:
            self.game.getRoot().queueEndRendering()

        # SPACE: lanzar tejo
        elif sym == OgreBites.SDLK_SPACE and not self.game.launching:
            self.game.launch_tejo()

        # R: reiniciar juego
        elif sym == OgreBites.SDLK_r:
            self.game.reset()

        # Flechas: ajustar ángulo vertical y posición X
        elif sym == OgreBites.SDLK_UP:
            delta = 5.0 if self.shift_pressed else 2.0
            self.game.angle_vert = min(80.0, self.game.angle_vert + delta)
            self.game.update_tejo_position()

        elif sym == OgreBites.SDLK_DOWN:
            delta = 5.0 if self.shift_pressed else 2.0
            self.game.angle_vert = max(5.0, self.game.angle_vert - delta)
            self.game.update_tejo_position()

        elif sym == OgreBites.SDLK_LEFT:
            delta = 0.5 if self.shift_pressed else 0.2
            self.game.launch_pos[0] = max(-6.0, self.game.launch_pos[0] - delta)
            self.game.update_tejo_position()

        elif sym == OgreBites.SDLK_RIGHT:
            delta = 0.5 if self.shift_pressed else 0.2
            self.game.launch_pos[0] = min(-0.5, self.game.launch_pos[0] + delta)
            self.game.update_tejo_position()

        # A/D: rotar horizontalmente
        elif sym == OgreBites.SDLK_a:
            delta = 5.0 if self.shift_pressed else 2.0
            self.game.angle_horiz = max(-30.0, self.game.angle_horiz - delta)
            self.game.update_tejo_position()

        elif sym == OgreBites.SDLK_d:
            delta = 5.0 if self.shift_pressed else 2.0
            self.game.angle_horiz = min(30.0, self.game.angle_horiz + delta)
            self.game.update_tejo_position()

        # + / - : ajustar potencia
        elif sym == OgreBites.SDLK_PLUS or sym == OgreBites.SDLK_EQUALS:
            delta = 1.0 if self.shift_pressed else 0.5
            self.game.power = min(20.0, self.game.power + delta)

        elif sym == OgreBites.SDLK_MINUS or sym == OgreBites.SDLK_UNDERSCORE:
            delta = 1.0 if self.shift_pressed else 0.5
            self.game.power = max(6.0, self.game.power - delta)

        return True

    def keyReleased(self, evt):
        sym = evt.keysym.sym
        if sym == OgreBites.SDLK_LSHIFT or sym == OgreBites.SDLK_RSHIFT:
            self.shift_pressed = False
        return True

    def mouseWheelRolled(self, evt):
        # Rueda del mouse → ajustar potencia
        delta = evt.y * 0.5
        self.game.power = max(6.0, min(20.0, self.game.power + delta))
        return True

    def mousePressed(self, evt):
        # Opcional: permite lanzar con el click
        print(f"Mouse button pressed: {evt.button}")
        return True