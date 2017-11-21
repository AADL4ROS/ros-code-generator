import logging

log_level_choices = {
        "INFO" : logging.INFO,
        "WARNING" : logging.WARNING,
        "ERROR" : logging.ERROR
    }

log_level = logging.INFO

def setup_custom_logger(name):
    global log_level
    formatter = logging.Formatter(fmt='%(levelname)s - %(module)s - %(message)s')

    handler = logging.StreamHandler()
    handler.setFormatter(formatter)

    logger = logging.getLogger(name)
    logger.setLevel( log_level )
    logger.addHandler(handler)
    return logger

def is_valid_log_level(level):
    global log_level_choices
    return (str(level).upper() in log_level_choices)

def set_log_level(level):
    global log_level_choices, log_level
    log_level = log_level_choices.get( str(level).upper(), "INFO" )