## @class AppError
# @brief Exception compliant with the ros error and trace srvs
class AppError(Exception):

  ## @brief Default constructor for passing error parameters
  def __init__(self, *args):
      # *args is used to get a list of the parameters passed in
      self.args = [a for a in args]
  
  ## @brief Validates the provided test type
  # @param e [AppError] The provided error
  # @param trace [Srv] Ros srv with error [string] and trace [string[]] arguments
  #
  # @return trace [Srv] Ros srv with error [string] and trace [string[]] arguments
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
  
