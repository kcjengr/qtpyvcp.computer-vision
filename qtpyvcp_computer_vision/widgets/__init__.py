from qtpyvcp.widgets.qtdesigner import _DesignerPlugin

from .opencv_widget import OpenCVWidget
class OpenCVWidget_Plugin(_DesignerPlugin):
    def pluginClass(self):
        return OpenCVWidget
