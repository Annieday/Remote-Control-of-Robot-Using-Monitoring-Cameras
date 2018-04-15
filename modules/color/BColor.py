# https://stackoverflow.com/questions/287871/print-in-terminal-with-colors
class BColor:
    ## color codes
    CYAN = '\033[96m'
    PURPLE = '\033[95m'
    BLUE = '\033[94m'
    YELLOW = '\033[93m'
    GREEN = '\033[92m'
    RED = '\033[91m'
    DARK_CYAN = '\033[36m'
    DARK_PURPLE = '\033[35m'
    DARK_BLUE = '\033[34m'
    DARK_YELLOW = '\033[33m'
    DARK_GREEN = '\033[32m'
    DARK_RED = '\033[31m'

    ## sytle codes
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

    def style(header, string):
        """ Adds a style header to string. """
        return header + string + BColor.ENDC

    def green(string):
        """ Turns a string green. """
        return BColor.style(BColor.GREEN, string)

    def red(string):
        """ Turns a string red. """
        return BColor.style(BColor.RED, string)

    def yellow(string):
        """ Turns a string blue. """
        return BColor.style(BColor.YELLOW, string)

    def cyan(string):
        """ Turns a string blue. """
        return BColor.style(BColor.CYAN, string)
