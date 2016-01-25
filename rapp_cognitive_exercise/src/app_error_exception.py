class AppError(Exception):
    def __init__(self, *args):
        # *args is used to get a list of the parameters passed in
        self.args = [a for a in args]
