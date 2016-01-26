class AppError(Exception):
  def __init__(self, *args):
      # *args is used to get a list of the parameters passed in
      self.args = [a for a in args]
  
  @staticmethod
  def passErrorToRosSrv(e,res):
    res.success=False
    if e.args[0] is not None and type(e.args[0]) is str:
      res.error=e.args[0]
    else:
      res.error="Unspecified application Error.."
    if (e.args[1] is not None and type(e.args[1]) is list):
      res.trace.extend(e.args[1])   
    elif(e.args[1] is not None and type(e.args[1]) is str):
      res.trace.append(e.args[1])
  
