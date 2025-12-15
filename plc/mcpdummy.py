class DummyPLC:
  def __init__(self):
    self._is_connected=True
  def connect(self,arg1,arg2):
    pass

  def batchread_wordunits(self,headdevice, readsize):
    return readsize*[0]

  def batchwrite_wordunits(self,headdevice, values):
    pass

