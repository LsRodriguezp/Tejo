## -------------------------------------------------------------------------
## @author Leonardo Florez-Valencia (florez-l@javeriana.edu.co)
## @modified by Sofía 
## -------------------------------------------------------------------------

import Ogre
import Ogre.Bites as OgreBites

class BaseListener(OgreBites.InputListener):

    m_Root = None

    def __init__(self, root):
        super(BaseListener, self).__init__()
        self.m_Root = root

    def keyPressed(self, evt):
        if evt.keysym.sym == OgreBites.SDLK_ESCAPE:
            self.m_Root.queueEndRendering()
        return True

    def mousePressed(self, evt):
        print(f'Mouse button pressed: {evt.button}')
        return True